"""IWR6843 serial driver — the host side of the L3-dump firmware contract.

Firmware v3+ speaks a SINGLE UART (the CP2105 Enhanced interface) at
1,041,667 baud for both CLI commands and the binary dump; the dump is framed
by its "ILD1" magic plus the header-declared length, so CLI echo and payload
can share the pipe. Hardware-validated 2026-07-13 at 100% of wire rate.

Gotchas baked in (each cost a debugging session):
- DTR/RTS must be held low on open (TI EVMs tie them to reset/boot mode).
- One serial handle only — two handles on one tty steal each other's bytes.
- The CP2105 can stall a stream for seconds (cp210x -110 control timeouts)
  and resume; the reader waits out gaps up to ``stall_tolerance_s`` (default
  8.0s -- raised from 4.0s after two range-session dumps came back short,
  507,450/510,508 and 440,594/441,348 bytes, consistent with a stall that
  outlasted the old tolerance mid-transfer). A dump lost to a stall this way
  cannot be re-requested: the firmware has already re-armed, and a fresh
  ``l3dump`` returns whatever is in the ring next, not the same capture.
"""

from __future__ import annotations

import glob
import logging
import time
from typing import Callable

import serial

from openflight.iwr6843.dump import HEADER, MAGIC, parse_header, payload_nbytes
from openflight.iwr6843.shot_result import parse_result_reply
from openflight.iwr6843.sparse import (
    POWER_MAGIC,
    SLICE_MAGIC,
    SPARSE_REQUEST_MAX_BYTES,
    TRACK_MAGIC,
    OnboardTrack,
    PowerSummary,
    SlicePlanner,
    SparseCapture,
    SparsePlan,
    assemble_capture,
    assemble_dump,
    fit_cell_request,
    format_cell_request,
    parse_power,
    parse_track,
    power_packet_size,
    slice_packet_size,
    track_packet_size,
)

BAUD = 1_041_667
# Firmware CLI line written when the self-trigger freezes the ring.
TRIGGER_NOTICE = b"Triggered"
# The firmware CLI prompt, written after every reply (l3_dump.c cliPrompt).
CLI_PROMPT = b"l3dump:/>"
_REPLY_VERDICTS = (b"Done", b"Error", b"not recognized")
# A reply whose prompt never comes is complete once its bytes stop for this
# long after the verdict (the firmware writes a reply without pauses).
_REPLY_QUIET_S = 0.1
_NOTICE_TAIL_BYTES = len(TRIGGER_NOTICE) - 1
_PORT_GLOBS = ("/dev/ttyUSB*", "/dev/tty.SLAB_USBtoUART*")

logger = logging.getLogger(__name__)


class UnsupportedCommand(RuntimeError):
    """The firmware CLI does not know the command, so it is an older image."""


def _reply_verdict_at(resp: bytes) -> int:
    """Index just past the first Done/Error/not-recognized word, or -1."""
    found = [(resp.index(word) + len(word)) for word in _REPLY_VERDICTS if word in resp]
    return min(found) if found else -1


def open_port(port: str, baud: int = BAUD, timeout: float = 0.3) -> serial.Serial:
    """DTR/RTS-safe serial open."""
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, timeout
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


class IWR6843Radar:
    """CLI + dump transport for the custom L3-dump firmware."""

    # A ``Triggered`` notice (or its split head) read by a command, kept for
    # the listener: the firmware stays frozen until someone releases it.
    _trigger_pending = b""

    def __init__(self, port: str | None = None, baud: int = BAUD):
        if port is None:
            port = self.detect_port(baud)
            if port is None:
                raise RuntimeError("no IWR6843 CLI found — board on, flashed, single-port fw?")
        self.port = port
        self.ser = open_port(port, baud)
        self._trigger_pending = b""
        self._last_cli_error = ""

    @staticmethod
    def detect_port(baud: int = BAUD) -> str | None:
        """First serial port whose CLI answers `help` with our commands."""
        candidates: list[str] = []
        for pattern in _PORT_GLOBS:
            candidates.extend(sorted(glob.glob(pattern)))
        for cand in candidates:
            try:
                ser = open_port(cand, baud)
            except (OSError, serial.SerialException):
                continue
            try:
                ser.reset_input_buffer()
                ser.write(b"help\n")
                resp = b""
                deadline = time.time() + 1.5
                while time.time() < deadline and b"sensorStart" not in resp:
                    resp += ser.read(512)
            finally:
                ser.close()
            if b"sensorStart" in resp:
                return cand
        return None

    def wait_trigger_notice(self, pending: bytes = b"") -> tuple[bool, bytes]:
        """Wait up to the port timeout for CLI bytes; report a ``Triggered`` line.

        ``read`` returns as soon as a byte arrives, so the notice is seen
        within about a millisecond. ``pending`` carries a partial line
        between calls; the tail kept is long enough to hold a split word.
        """
        pending = self._trigger_pending + pending
        self._trigger_pending = b""
        if TRIGGER_NOTICE not in pending:
            waiting = self.ser.in_waiting
            pending += self.ser.read(waiting if waiting else 1)
        if TRIGGER_NOTICE in pending:
            return True, b""
        return False, pending[-_NOTICE_TAIL_BYTES:]

    def _remember_trigger_notice(self, data: bytes) -> None:
        """Keep a notice (or its split head) that a command read off the port."""
        pending = self._trigger_pending + data
        if TRIGGER_NOTICE in pending:
            self._trigger_pending = TRIGGER_NOTICE
        else:
            self._trigger_pending = pending[-_NOTICE_TAIL_BYTES:]

    def _discard_before_readback(self) -> None:
        """Drop stale input before reading the frozen capture out.

        A notice pending here belongs to the capture this readback consumes,
        so keeping it would fire a phantom capture once the ring rearms.
        """
        self.ser.reset_input_buffer()
        self._trigger_pending = b""

    def cmd(self, line: str, window: float = 1.5) -> str:
        """Send one CLI line; collect the reply through the prompt that ends it.

        The reply is complete at the CLI prompt that follows its Done, Error
        or "not recognized" line, not at the first of those words: the
        firmware writes a reply byte by byte, so a read can return
        ``Error: stop the senso`` and the rest would otherwise arrive as the
        next command's reply. Without a prompt the window bounds the wait.
        Stale bytes are dropped rather than taken as the reply, except a
        ``Triggered`` notice among them, which is kept for the listener.
        """
        waiting = self.ser.in_waiting
        if waiting:
            self._remember_trigger_notice(self.ser.read(waiting))
        self.ser.write((line + "\n").encode())
        resp = b""
        verdict_at = -1
        reply_end = -1
        deadline = time.time() + window
        last_byte_at = time.time()
        while time.time() < deadline:
            # in_waiting-sized reads, else read(1): read(512) would wait out
            # the port timeout for every reply shorter than 512 bytes.
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting if waiting else 1)
            if chunk:
                resp += chunk
                last_byte_at = time.time()
            if verdict_at < 0:
                verdict_at = _reply_verdict_at(resp)
            if verdict_at < 0:
                continue
            if CLI_PROMPT in resp[verdict_at:]:
                reply_end = resp.index(CLI_PROMPT, verdict_at) + len(CLI_PROMPT)
                break
            if time.time() - last_byte_at >= _REPLY_QUIET_S:
                break
        # The whole read is scanned for a notice, split or not; only the reply
        # is returned. Bytes after the prompt are the start of whatever
        # streams next (a debug line, a notice), and a partial debug line
        # handed back here would parse as one with fields missing.
        self._remember_trigger_notice(resp)
        if reply_end >= 0:
            resp = resp[:reply_end]
        return resp.decode(errors="replace")

    def drain_stale_output(
        self,
        *,
        max_wait_s: float = 10.0,
        initial_quiet_s: float = 0.25,
        stream_quiet_s: float = 4.25,
    ) -> int:
        """Drain an abandoned binary dump before sending configuration commands.

        If a host process exits during ``l3dump``, the firmware can still be
        writing the old payload through the CP2105. Commands sent into that
        stream are not safe to associate with their responses. Once bytes are
        observed, tolerate the bridge's known multi-second stalls before
        declaring the stream quiet.
        """
        drained = 0
        saw_data = False
        start = time.monotonic()
        last_data = start
        while time.monotonic() - start < max_wait_s:
            waiting = self.ser.in_waiting
            if waiting:
                chunk = self.ser.read(min(waiting, 4096))
                if chunk:
                    drained += len(chunk)
                    saw_data = True
                    last_data = time.monotonic()
                    continue
            quiet_s = stream_quiet_s if saw_data else initial_quiet_s
            if time.monotonic() - last_data >= quiet_s:
                break
            time.sleep(0.01)
        if drained:
            logger.warning(
                "[IWR6843] Drained %d stale UART bytes before configuration",
                drained,
            )
        return drained

    @staticmethod
    def _require_done(command: str, response: str) -> None:
        if "Error" in response:
            raise RuntimeError(f"config rejected: {command!r}: {response.strip()}")
        if "Done" not in response:
            raise RuntimeError(
                f"IWR6843 did not acknowledge {command!r}; "
                "the firmware may be wedged (press RESET and retry)"
            )

    def send_config(self, cfg_path: str) -> None:
        """Stop and flush old state, then stream the cfg; raise on Error.

        The firmware's geometry guard rejects a cfg whose loops/samples don't
        match the flashed build — that surfaces here as RuntimeError.
        """
        self.drain_stale_output()
        # A crashed watch session can leave debug lines streaming, or the CLI
        # blocked in l3sparse's cell-request read. This line silences debug
        # and, if that read is still open, completes it so sensorStop is a
        # real command.
        self.cmd("debugCfg 0", 8.0)
        self._require_done("sensorStop", self.cmd("sensorStop", 3.0))
        self._require_done("flushCfg", self.cmd("flushCfg", 1.5))
        self._trigger_pending = b""
        with open(cfg_path, encoding="utf-8") as cfg:
            for rawline in cfg:
                line = rawline.strip()
                if not line or line.startswith("%"):
                    continue
                # The driver owns the lifecycle commands so every config gets
                # the required stop/flush ordering without sending duplicates.
                if line in {"sensorStop", "flushCfg", "debugCfg 0"}:
                    continue
                # sensorStart blocks on RF calibration; 6s loses a cold BSS.
                window = 12.0 if line.startswith("sensorStart") else 1.5
                resp = self.cmd(line, window)
                self._require_done(line, resp)
        deadline = time.monotonic() + 6.0
        health = ""
        while time.monotonic() < deadline:
            health = self.stats()
            self._require_done("stats", health)
            if "active=1" in health:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(f"IWR6843 did not enter active capture mode: {health.strip()}")

    def read_dump(self, timeout_s: float = 40.0, stall_tolerance_s: float = 8.0) -> bytes:
        """Fire `l3dump` and return one complete dump (best effort on stalls).

        Syncs on the ILD1 magic past the CLI echo and sizes the read from the
        dump's own header, so any firmware geometry works.
        """
        self._discard_before_readback()
        self.ser.write(b"l3dump\n")
        buf = bytearray()
        expected: int | None = None
        start = time.time()
        last = start
        while time.time() - start < timeout_s:
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting if waiting else 1)
            if chunk:
                buf.extend(chunk)
                last = time.time()
            elif buf and time.time() - last > stall_tolerance_s:
                # This fires the instant the gap crosses stall_tolerance_s, so
                # the elapsed time logged here is always ~stall_tolerance_s --
                # it does NOT reveal how long the real gap was, only that it
                # was at least this long.
                logger.warning(
                    "[IWR6843] Dump stream stalled >= %.1fs, giving up (%d/%s bytes received)",
                    stall_tolerance_s,
                    len(buf),
                    expected,
                )
                break
            if expected is None:
                idx = buf.find(MAGIC)
                if idx >= 0 and len(buf) - idx >= HEADER.size:
                    del buf[:idx]
                    try:
                        metadata = parse_header(buf)
                        expected = metadata["header_nbytes"] + payload_nbytes(metadata, buf)
                    except ValueError:
                        expected = None
            elif len(buf) >= expected:
                break
        if expected is None:
            return bytes(buf)

        payload = bytes(buf[:expected])
        if len(payload) == expected:
            # The binary payload can finish just before the CLI handler returns.
            # Wait for its trailing Done before another command can be consumed
            # by the firmware while it is still completing dump/restart work.
            elapsed = time.time() - start
            trailer = self._wait_for_dump_cli_ready(
                buf[expected:], timeout_s=min(1.0, max(0.0, timeout_s - elapsed))
            )
            if b"Error" in trailer:
                raise RuntimeError(
                    f"IWR6843 dump completed but firmware restart failed: "
                    f"{trailer.decode(errors='replace').strip()}"
                )
        return payload

    def read_sparse(self, planner: SlicePlanner, timeout_s: float = 8.0) -> SparseCapture | None:
        """Freeze, read residual power, then the complex cells ``planner`` names.

        Returns None only when the firmware rejects ``l3sparse`` before it
        freezes (older firmware, IQ8 storage), so ``l3dump`` is still safe.
        Any failure after that raises: the firmware has already re-armed, and
        an ``l3dump`` would return a ring recorded after the shot.
        """
        try:
            exchange = self._sparse_exchange(planner, timeout_s)
        except UnsupportedCommand:
            return None
        if exchange is None:
            return None
        summary, plan, requested, slice_packet = exchange
        return assemble_capture(summary, slice_packet, plan, requested_cells=requested)

    def release_sparse_freeze(self, timeout_s: float = 8.0) -> None:
        """Rearm a self-triggered freeze.

        ``l3sparse`` reads its cell line only after streaming the power map.
        The SCI receiver holds one byte, so a line written during that stream
        is overwritten, and writing it afterwards loses to the 5s firmware
        wait when the CP2105 stalls. Release asks for no cells, so it is one
        command.
        """
        self._discard_before_readback()
        reply = self.cmd("l3release", timeout_s)
        if "not recognized" in reply:
            raise RuntimeError(
                "IWR6843 has no l3release; flash the current firmware to clear a self-trigger"
            )
        if "Error" in reply or "Done" not in reply:
            detail = reply.strip() or "no acknowledgement"
            raise RuntimeError(f"IWR6843 did not release the frozen ring: {detail}")

    def read_tracked(self, timeout_s: float = 8.0) -> tuple[bytes, float, OnboardTrack] | None:
        """Freeze and read the cells the firmware tracker chose (``l3track``).

        Returns the assembled dump, the noise power and the firmware's track,
        or None when the firmware refuses before streaming (no trackCfg, IQ8
        storage), so the caller can fall back to another command.
        Raises UnsupportedCommand when the firmware has no ``l3track``, and
        RuntimeError when the stream breaks after it starts: the ring has
        already been rearmed, so a fallback would capture the wrong window.
        """
        self._discard_before_readback()
        self.ser.write(b"l3track\n")
        try:
            read = self._read_packet(TRACK_MAGIC, track_packet_size, timeout_s)
        except TimeoutError as exc:
            raise RuntimeError("IWR6843 l3track packet ended early") from exc
        if read is None:
            return None
        packet, rest = read
        layout, track = parse_track(packet)
        try:
            read = self._read_packet(
                SLICE_MAGIC,
                lambda header: slice_packet_size(header, layout),
                timeout_s,
                pending=rest,
            )
        except TimeoutError as exc:
            raise RuntimeError("IWR6843 l3track cell packet ended early") from exc
        if read is None:
            raise RuntimeError("IWR6843 l3track cell packet ended early")
        slice_packet, rest = read
        trailer = self._wait_for_dump_cli_ready(rest, timeout_s=1.0)
        if b"Error" in trailer:
            raise RuntimeError(
                "IWR6843 track capture completed but firmware restart failed: "
                f"{trailer.decode(errors='replace').strip()}"
            )
        return assemble_dump(layout, slice_packet), layout.noise_power, track

    def _sparse_exchange(
        self,
        planner: SlicePlanner,
        timeout_s: float,
    ) -> tuple[PowerSummary, SparsePlan, int, bytes] | None:
        """Run one l3sparse round trip. None when rejected before the freeze."""
        self._discard_before_readback()
        self._last_cli_error = ""
        self.ser.write(b"l3sparse\n")
        read = self._read_packet(POWER_MAGIC, power_packet_size, timeout_s)
        if read is None:
            return None
        packet, rest = read
        summary = parse_power(packet)
        try:
            plan = planner(summary)
        except Exception:
            # The firmware is frozen and reading its next CLI line as the
            # cell request. Answer it, or the next command would be eaten.
            self.ser.write(format_cell_request([]))
            raise
        ordered = plan.request_order()
        request, sent = fit_cell_request(ordered)
        if sent < len(ordered):
            logger.warning(
                "[IWR6843] Sparse request trimmed to %d of %d cells (%d-byte limit)",
                sent,
                len(ordered),
                SPARSE_REQUEST_MAX_BYTES,
            )
        self.ser.write(request)
        read = self._read_packet(
            SLICE_MAGIC,
            lambda header: slice_packet_size(header, summary),
            timeout_s,
            pending=rest,
        )
        if read is None:
            detail = self._last_cli_error
            message = "IWR6843 rejected the sparse cell request after freezing"
            if detail:
                message = f"{message}: {detail}"
            raise RuntimeError(message)
        slice_packet, rest = read
        trailer = self._wait_for_dump_cli_ready(rest, timeout_s=1.0)
        if b"Error" in trailer:
            raise RuntimeError(
                "IWR6843 sparse capture completed but firmware restart failed: "
                f"{trailer.decode(errors='replace').strip()}"
            )
        return summary, plan, len(ordered), slice_packet

    def _read_packet(
        self,
        magic: bytes,
        packet_size: Callable[[bytes], int],
        timeout_s: float,
        pending: bytes = b"",
    ) -> tuple[bytes, bytes] | None:
        """Read one ``magic`` packet sized by its own header.

        Returns (packet, bytes after it), or None when the CLI reports an
        error before the magic. Only bytes before the magic are searched for
        error text; the binary payload can contain any byte sequence.
        ``pending`` holds bytes already read that belong to this packet.
        """
        header_size = packet_size(b"")
        buf = bytearray(pending)
        deadline = time.monotonic() + timeout_s
        start: int | None = None
        while time.monotonic() < deadline:
            if start is None:
                idx = buf.find(magic)
                if idx < 0:
                    text = bytes(buf)
                    if b"not recognized" in text:
                        raise UnsupportedCommand(text.decode(errors="replace").strip())
                    error_at = text.find(b"Error")
                    if error_at >= 0 and b"\n" in text[error_at:]:
                        self._last_cli_error = text.decode(errors="replace").strip()
                        return None
                else:
                    start = idx
            if start is not None:
                body = bytes(buf[start:])
                if len(body) >= header_size:
                    total = packet_size(body)
                    if len(body) >= total:
                        return body[:total], body[total:]
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting if waiting else 1)
            if chunk:
                buf.extend(chunk)
        if start is None and b"Error" in buf:
            self._last_cli_error = bytes(buf).decode(errors="replace").strip()
            return None
        raise TimeoutError(
            f"IWR6843 {magic.decode()} packet incomplete after {timeout_s:.1f}s ({len(buf)} bytes)"
        )

    def _wait_for_dump_cli_ready(self, initial: bytes, *, timeout_s: float) -> bytes:
        """Consume the dump handler's trailing response before reusing the CLI."""
        response = bytearray(initial)
        deadline = time.monotonic() + timeout_s
        while b"Done" not in response and b"Error" not in response:
            if time.monotonic() >= deadline:
                break
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting if waiting else 1)
            if chunk:
                response.extend(chunk)
        self._remember_trigger_notice(bytes(response))
        return bytes(response)

    def stats(self) -> str:
        """Firmware health line (frames/wraps/active/calib/rf_faults)."""
        return self.cmd("stats", 2.0)

    def trigger_log(self) -> str:
        """The self-trigger detector's summary, configuration and frame log.

        One text line per logged frame: candidate bin, integrated residual
        energy against the noise floor, Doppler velocity and coherence, track
        age, and why the frame did or did not fire. A missed swing reads back
        as long as nothing above the floor has been seen since; the firmware
        prints up to 128 records, so allow a few seconds at CLI baud.
        """
        return self.cmd("triggerLog", 6.0)

    def tee_scan(self, first_bin: int, count: int) -> str:
        """``ball scan``: static power per global bin averaged over the ring's pre frames.

        The firmware freezes at the next frame boundary, reports, and rearms,
        so the sensor must be active. Parse with ``tee_scan.parse_tee_scan``.
        """
        return self.cmd(f"ball scan {first_bin} {count}", 4.0)

    def ball_status(self) -> str:
        """The ball-placement detector's state line; parse with ``tee_scan.parse_ball_status``."""
        return self.cmd("ball status", 2.0)

    def configure_ball(self, enable: bool, follow: bool) -> None:
        """Start (or stop) the ball-placement detector; ``follow`` aims the trigger at it."""
        self._require_done(
            "ball cfg", self.cmd(f"ball cfg {int(enable)} {int(follow and enable)}", 2.0)
        )

    def trigger_trace(self) -> str:
        """The detector's raw-input trace: what it was offered, not what it took.

        A header, the per-bin maximum of the detection statistic since arming
        (``bin:max@frame``), then one line per frame whose strongest bin
        reached twice the floor, with all-loop energy, strongest-loop peak,
        loop-0 power and the floor. Read it after a missed swing: an empty
        trace with flat maxima means the club was not seen at all; entries
        under ``floor x snr`` mean it was seen and thresholded out.
        """
        return self.cmd("triggerLog trace", 6.0)

    def clear_trigger_trace(self) -> None:
        """Empty the trace and its maxima; the arm and the log are untouched."""
        self._require_done("triggerLog clear", self.cmd("triggerLog clear", 2.0))

    def club_track(self) -> str:
        """``triggerLog track``: the club track, its delivery fit, the newest
        angle estimate, the geometric impact verdict and every held point."""
        return self.cmd("triggerLog track", 3.0)

    def shot_status(self) -> str:
        """``triggerLog shot``: the shot machine, the ball track, the launch and
        the ball points."""
        return self.cmd("triggerLog shot", 3.0)

    def shot_result(self):
        """The firmware's result packet for the last shot, parsed, or None when
        the machine has not reached RESULT since the last rearm.

        Reads ``triggerLog result``; the packet lines are always printed (the
        last shot's, or all invalid), so ``ready=0`` on the first line is what
        says there is no result yet.
        """
        reply = self.cmd("triggerLog result", 3.0)
        if " ready=1" not in reply:
            return None
        return parse_result_reply(reply)

    def perf(self) -> str:
        """``triggerLog perf``: per-stage microseconds and the adaptive window state."""
        return self.cmd("triggerLog perf", 2.0)

    def stop_sensor(self) -> None:
        """Stop capture and verify the firmware returned to its idle CLI state."""
        self._require_done("sensorStop", self.cmd("sensorStop", 3.0))
        health = self.stats()
        self._require_done("stats", health)
        if "active=0" not in health:
            raise RuntimeError(f"IWR6843 remained active after sensorStop: {health.strip()}")

    def close(self) -> None:
        """Release the serial port."""
        self.ser.close()

    def __enter__(self) -> "IWR6843Radar":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
