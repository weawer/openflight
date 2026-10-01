"""Parse the MSS <-> DSS detect link diagnostics (firmware ``trackCfg dsp``).

The detect task is moving from the R4F (MSS) to the C674x (DSS). Phase 0
proves the link on the board: ``trackCfg dsp ping`` (the DSS answers over
the mailbox) and ``trackCfg dsp probe [bins]`` (the newest ring frame scored
on both cores with the same code, l3_bin_score.c, timed and compared bit
for bit).

Then the live detector: ``trackCfg detectCore [dss|verify]`` chooses how
each frame's bins are scored (firmware l3_detect_core.h; the MSS scores
only the frames the DSS cannot take, and every frame once latched) and prints
the ``detect core=...`` line; ``triggerLog perf`` and ``triggerLog timing``
print it with the detect timing (l3_timing.h), which keeps latency and
throughput apart.
"""

from __future__ import annotations

import re
import statistics
from dataclasses import dataclass

_PONG = re.compile(r"^dsp pong seq=(\d+) us=(\d+)\s*$", re.MULTILINE)
_PROBE = re.compile(
    r"^dsp probe slot=(?P<slot>\d+) bins=(?P<bins>\d+) mss_us=(?P<mss_us>\d+) "
    r"dss_us=(?P<dss_us>\d+) dss_cycles=(?P<dss_cycles>\d+) match=(?P<match>[01]) "
    r"status=(?P<status>\d+) mss_energy=(?P<mss_energy>[0-9a-f]{8}) "
    r"dss_energy=(?P<dss_energy>[0-9a-f]{8})"
    r"(?: dss_prep_us=(?P<dss_prep_us>\d+) gathered=(?P<gathered>[01]))?\s*$",
    re.MULTILINE,
)
_ERROR = re.compile(r"^Error:\s*(.*)$", re.MULTILINE)
_STATUS = re.compile(
    r"^dsp status stage=(?P<stage>\w+)(?P<failed> FAILED)? err=(?P<err>-?\d+) "
    r"beats=(?P<beats>\d+) served=(?P<served>\d+)"
    r"(?: exc_pc=(?P<exc_pc>[0-9a-f]{8}) exc_efr=(?P<exc_efr>[0-9a-f]{8}))?\s*$",
    re.MULTILINE,
)
_HW = re.compile(
    r"^dsp hw gpreg_stage=(?P<stage>\S+) halt=(?P<halt>\d+) power=(?P<power>\d+) "
    r"stc=(?P<stc>\d+) esm=(?P<esm>[0-9a-f]{8}(?:,[0-9a-f]{8}){3}) "
    r"hsram=(?P<hsram>ok|BAD)\s*$",
    re.MULTILINE,
)
_NEVER_BOOTED = re.compile(r"^dsp status stage=never_booted magic=[0-9a-f]{8}\s*$", re.MULTILINE)
_CORES = r"(?:mss|dss|verify)"
_DETECT = re.compile(
    rf"^detect core=(?P<requested>{_CORES}) active=(?P<active>{_CORES}) "
    r"latched=(?P<latched>[01]) mss=(?P<mss>\d+) dss=(?P<dss>\d+) verify=(?P<verify>\d+) "
    r"ineligible=(?P<ineligible>\d+) failures=(?P<failures>\d+) fallbacks=(?P<fallbacks>\d+) "
    r"streak=(?P<streak>\d+) latches=(?P<latches>\d+) mismatches=(?P<mismatches>\d+) "
    r"dss_inv_us=(?P<inv_last>\d+)/(?P<inv_max>\d+) "
    r"dss_score_us=(?P<score_last>\d+)/(?P<score_max>\d+)"
    r"(?: first_mismatch=(?P<mm_slot>\d+):(?P<mm_bin>\d+):(?P<mm_field>\d+))?\s*$",
    re.MULTILINE,
)
_TIMING_SUMMARY = re.compile(
    r"^timing frames=(?P<frames>\d+) budget_us=(?P<budget>\d+) "
    r"over_budget=(?P<over>\d+) depth_max=(?P<depth>\d+) ring=(?P<ring>\d+) "
    r"margin_last_us=(?P<margin_last>-?\d+|-) margin_min_us=(?P<margin_min>-?\d+|-) "
    r"margin_negative=(?P<negative>\d+)\s*$",
    re.MULTILINE,
)
_TIMING_NONE = re.compile(r"^timing frames=0 \(no frame decided yet\)\s*$", re.MULTILINE)
_TIMING_STAT = re.compile(
    r"^timing (?P<name>wait|score|service|latency|arrival) n=(?P<n>\d+) last=(?P<last>\d+) "
    r"min=(?P<min>\d+) mean=(?P<mean>\d+) max=(?P<max>\d+)\s*$",
    re.MULTILINE,
)
_TIMELINE = re.compile(
    r"^timeline slot=(?P<slot>\d+) epoch=(?P<epoch>\d+|post) "
    r"core=(?P<core>mss|dss|verify|fallback) wait_us=(?P<wait>\d+) score_us=(?P<score>\d+|-) "
    r"service_us=(?P<service>\d+) latency_us=(?P<latency>\d+) depth=(?P<depth>\d+) "
    r"flags=(?P<flags>[a-z|]+|-)\s*$",
    re.MULTILINE,
)
# What ``trackCfg detectCore`` accepts. mss is not one: the status line
# still prints it as the active core once latched (``_CORES``).
DETECT_CORES = ("dss", "verify")
# l3_dsp_ipc.h L3_DSP_FIELD_*: which field of a bin two cores disagreed on.
MISMATCH_FIELDS = ("energy", "peak", "loop0", "r1Re", "r1Im", "set")


class DspLinkError(RuntimeError):
    """The firmware refused the command, the DSS did not answer, or the
    image has no detect link (an older image refuses ``trackCfg dsp``)."""


@dataclass(frozen=True)
class DspProbe:
    """One ``trackCfg dsp probe``: the same frame scored on both cores."""

    slot: int
    bins: int
    mss_us: int
    dss_us: int
    dss_cycles: int
    match: bool  # every sum bit for bit equal
    status: int  # the DSS's L3_DSP_* status
    mss_energy: str  # the energy sum's float bits, hex
    dss_energy: str
    # Images with the gather: what preparing the frame cost the DSS before
    # scoring (the gather into L2, or the invalidate over L3), and which.
    dss_prep_us: int | None = None
    gathered: bool | None = None

    @property
    def dss_total_us(self) -> int:
        """What the DSS spent on the frame: preparing it, then scoring."""
        return self.dss_us + (self.dss_prep_us or 0)


@dataclass(frozen=True)
class DspStatus:
    """``trackCfg dsp status``: the boot stage the DSS reached (HS-RAM).

    Stages in order: reset, startup_first, startup_last (the xdc/BIOS
    module startups run between those two), main, soc_init, task,
    mailbox_init, link_open (serving); exception when the DSS took one
    (``exc_pc`` the interrupted program counter, ``exc_efr`` the flags);
    never_booted when the DSS never wrote its status. ``beats`` rises while
    the link task waits for requests, so a rising count means BIOS runs.
    """

    stage: str
    failed: bool
    err: int
    beats: int
    served: int
    exc_pc: int | None = None
    exc_efr: int | None = None

    @property
    def booted(self) -> bool:
        return self.stage != "never_booted"


@dataclass(frozen=True)
class DspHw:
    """``trackCfg dsp hw``: the DSS as the MSS reads it without the DSS's help.

    ``gpreg_stage`` is the stage the DSS mirrored into DSSGPREG0 (``reset``
    is its earliest, before C init and BIOS), or ``none(XXXXXXXX)`` when it
    never wrote it; ``esm`` the ESMSR1..3 and ESMSR4 error flags.
    """

    gpreg_stage: str
    halt: int
    power: int
    stc: int
    esm: tuple[int, int, int, int]
    hsram_ok: bool

    @property
    def halted(self) -> bool:
        return self.halt == 1

    @property
    def powered(self) -> bool:
        return self.power == 3


@dataclass(frozen=True)
class DspProbeSummary:
    count: int
    mismatches: int
    mss_us_median: float
    dss_us_median: float  # scoring
    dss_total_us_median: float = 0.0  # preparing and scoring
    gathered: int = 0  # probes the DSS scored from the gathered copy

    @property
    def speedup(self) -> float:
        """How many times faster the DSS handled the same bins, its
        preparation (the gather) counted."""
        total = self.dss_total_us_median or self.dss_us_median
        return self.mss_us_median / total if total else float("inf")


@dataclass(frozen=True)
class DetectMismatch:
    """The first frame ``verify`` found the cores disagreeing on."""

    slot: int
    bin: int  # local bin
    field: str  # MISMATCH_FIELDS


@dataclass(frozen=True)
class DetectCoreStatus:
    """The ``detect core=...`` line: which core scores, and how it went.

    ``requested`` is what was chosen; ``active`` is where frames go now,
    ``mss`` once three DSS failures in a row ``latched`` it. ``ineligible``
    frames (not an IQ16 ring frame, or the link busy with a CLI command) and
    ``fallbacks`` (the DSS failed the frame) were scored on the MSS.
    """

    requested: str
    active: str
    latched: bool
    mss: int
    dss: int
    verify: int
    ineligible: int
    failures: int
    fallbacks: int
    streak: int
    latches: int
    mismatches: int
    dss_inv_us_last: int
    dss_inv_us_max: int
    dss_score_us_last: int
    dss_score_us_max: int
    first_mismatch: DetectMismatch | None


@dataclass(frozen=True)
class TimingStat:
    """One ``timing <name>`` line, microseconds."""

    count: int
    last: int
    min: int
    mean: int
    max: int


@dataclass(frozen=True)
class TimelineEvent:
    """One ``timeline`` line: a frame's life on the detect task."""

    slot: int
    epoch: int | None  # None for a post-impact frame
    core: str  # mss, dss, verify, or fallback
    wait_us: int
    score_us: int | None  # None when the frame was not scored
    service_us: int
    latency_us: int
    depth: int
    flags: frozenset[str]


@dataclass(frozen=True)
class DetectTiming:
    """``triggerLog perf`` / ``triggerLog timing``: the two deadlines.

    Throughput: ``service`` must average below ``budget_us`` (the frame
    interval); ``over_budget`` counts frames whose service did not.
    Latency: a frame may take longer than the interval as long as its ring
    slot is not reused first; ``margin_min_us`` is the tightest that came,
    ``margin_negative`` the frames that ran past it (None before any).
    """

    frames: int
    budget_us: int
    over_budget: int
    depth_max: int
    ring: int
    margin_last_us: int | None
    margin_min_us: int | None
    margin_negative: int
    stats: dict[str, TimingStat]
    timeline: tuple[TimelineEvent, ...]

    @property
    def keeps_up(self) -> bool:
        """Mean service within the frame interval and no slot overrun."""
        service = self.stats.get("service")
        return (
            service is not None
            and service.count > 0
            and service.mean < self.budget_us
            and self.margin_negative == 0
        )


def _raise_on_error(text: str, what: str) -> None:
    error = _ERROR.search(text)
    if error:
        raise DspLinkError(f"{what}: {error.group(1).strip()}")


def parse_dsp_pong(text: str) -> int:
    """The ping's round trip in microseconds."""
    _raise_on_error(text, "trackCfg dsp ping")
    match = _PONG.search(text)
    if match is None:
        raise DspLinkError(f"trackCfg dsp ping: no pong in {text!r}")
    return int(match.group(2))


def parse_dsp_probe(text: str) -> DspProbe:
    """A probe line. A mismatch is data (``match`` False), not an error."""
    _raise_on_error(text, "trackCfg dsp probe")
    match = _PROBE.search(text)
    if match is None:
        raise DspLinkError(f"trackCfg dsp probe: no probe line in {text!r}")
    fields = match.groupdict()
    return DspProbe(
        slot=int(fields["slot"]),
        bins=int(fields["bins"]),
        mss_us=int(fields["mss_us"]),
        dss_us=int(fields["dss_us"]),
        dss_cycles=int(fields["dss_cycles"]),
        match=fields["match"] == "1",
        status=int(fields["status"]),
        mss_energy=fields["mss_energy"],
        dss_energy=fields["dss_energy"],
        dss_prep_us=None if fields["dss_prep_us"] is None else int(fields["dss_prep_us"]),
        gathered=None if fields["gathered"] is None else fields["gathered"] == "1",
    )


def parse_dsp_status(text: str) -> DspStatus:
    """The status line; also found in a failed ping's or probe's reply,
    which prints it before its Error line."""
    if _NEVER_BOOTED.search(text):
        return DspStatus(stage="never_booted", failed=False, err=0, beats=0, served=0)
    match = _STATUS.search(text)
    if match is None:
        _raise_on_error(text, "trackCfg dsp status")
        raise DspLinkError(f"trackCfg dsp status: no status line in {text!r}")
    return DspStatus(
        stage=match.group("stage"),
        failed=match.group("failed") is not None,
        err=int(match.group("err")),
        beats=int(match.group("beats")),
        served=int(match.group("served")),
        exc_pc=None if match.group("exc_pc") is None else int(match.group("exc_pc"), 16),
        exc_efr=None if match.group("exc_efr") is None else int(match.group("exc_efr"), 16),
    )


def parse_dsp_hw(text: str) -> DspHw:
    """The hardware line; also found in a failed ping's or probe's reply."""
    match = _HW.search(text)
    if match is None:
        _raise_on_error(text, "trackCfg dsp hw")
        raise DspLinkError(f"trackCfg dsp hw: no hw line in {text!r}")
    esm = tuple(int(word, 16) for word in match.group("esm").split(","))
    return DspHw(
        gpreg_stage=match.group("stage"),
        halt=int(match.group("halt")),
        power=int(match.group("power")),
        stc=int(match.group("stc")),
        esm=(esm[0], esm[1], esm[2], esm[3]),
        hsram_ok=match.group("hsram") == "ok",
    )


def summarize_probes(probes: list[DspProbe]) -> DspProbeSummary:
    """Median timings over repeated probes, and how many disagreed."""
    if not probes:
        raise ValueError("no probes to summarize")
    return DspProbeSummary(
        count=len(probes),
        mismatches=sum(not probe.match for probe in probes),
        mss_us_median=statistics.median(probe.mss_us for probe in probes),
        dss_us_median=statistics.median(probe.dss_us for probe in probes),
        dss_total_us_median=statistics.median(probe.dss_total_us for probe in probes),
        gathered=sum(bool(probe.gathered) for probe in probes),
    )


def parse_detect_core(text: str) -> DetectCoreStatus:
    """The ``detect core=...`` line of ``trackCfg detectCore``, ``triggerLog
    perf`` or ``triggerLog timing``."""
    _raise_on_error(text, "trackCfg detectCore")
    match = _DETECT.search(text)
    if match is None:
        raise DspLinkError(f"trackCfg detectCore: no detect line in {text!r}")
    fields = match.groupdict()
    mismatch = None
    if fields["mm_slot"] is not None:
        field = int(fields["mm_field"])
        mismatch = DetectMismatch(
            slot=int(fields["mm_slot"]),
            bin=int(fields["mm_bin"]),
            field=MISMATCH_FIELDS[field] if field < len(MISMATCH_FIELDS) else str(field),
        )
    return DetectCoreStatus(
        requested=fields["requested"],
        active=fields["active"],
        latched=fields["latched"] == "1",
        mss=int(fields["mss"]),
        dss=int(fields["dss"]),
        verify=int(fields["verify"]),
        ineligible=int(fields["ineligible"]),
        failures=int(fields["failures"]),
        fallbacks=int(fields["fallbacks"]),
        streak=int(fields["streak"]),
        latches=int(fields["latches"]),
        mismatches=int(fields["mismatches"]),
        dss_inv_us_last=int(fields["inv_last"]),
        dss_inv_us_max=int(fields["inv_max"]),
        dss_score_us_last=int(fields["score_last"]),
        dss_score_us_max=int(fields["score_max"]),
        first_mismatch=mismatch,
    )


def _margin(value: str) -> int | None:
    return None if value == "-" else int(value)


def parse_detect_timing(text: str) -> DetectTiming | None:
    """The timing lines of ``triggerLog perf`` or ``triggerLog timing``;
    None when no frame has been decided yet this session."""
    _raise_on_error(text, "triggerLog timing")
    if _TIMING_NONE.search(text):
        return None
    summary = _TIMING_SUMMARY.search(text)
    if summary is None:
        raise DspLinkError(f"triggerLog timing: no timing summary in {text!r}")
    stats = {
        match.group("name"): TimingStat(
            count=int(match.group("n")),
            last=int(match.group("last")),
            min=int(match.group("min")),
            mean=int(match.group("mean")),
            max=int(match.group("max")),
        )
        for match in _TIMING_STAT.finditer(text)
    }
    timeline = tuple(
        TimelineEvent(
            slot=int(match.group("slot")),
            epoch=None if match.group("epoch") == "post" else int(match.group("epoch")),
            core=match.group("core"),
            wait_us=int(match.group("wait")),
            score_us=None if match.group("score") == "-" else int(match.group("score")),
            service_us=int(match.group("service")),
            latency_us=int(match.group("latency")),
            depth=int(match.group("depth")),
            flags=frozenset()
            if match.group("flags") == "-"
            else frozenset(match.group("flags").split("|")),
        )
        for match in _TIMELINE.finditer(text)
    )
    return DetectTiming(
        frames=int(summary.group("frames")),
        budget_us=int(summary.group("budget")),
        over_budget=int(summary.group("over")),
        depth_max=int(summary.group("depth")),
        ring=int(summary.group("ring")),
        margin_last_us=_margin(summary.group("margin_last")),
        margin_min_us=_margin(summary.group("margin_min")),
        margin_negative=int(summary.group("negative")),
        stats=stats,
        timeline=timeline,
    )


# --- the board acceptance run --------------------------------------------------------
#
# scripts/hardware-test/iwr6843_dsp_probe.py --acceptance: after the probe,
# the detector scores in ``verify`` (both cores, compared) and then in ``dss``
# while the operator swings; these judge what the board then reports.

_HEALTH = re.compile(
    r"^detect dropped=(?P<dropped>\d+) stale=(?P<stale>\d+) notice_dropped=(?P<notice>\d+) "
    r"shed=(?P<shed>\d+) stale_read=(?P<stale_read>\d+)\s*$",
    re.MULTILINE,
)
_ANGLES = re.compile(
    r"^angles queued=(?P<queued>\d+) done=(?P<done>\d+) stale=(?P<stale>\d+) "
    r"failed=(?P<failed>\d+) dropped=(?P<dropped>\d+) pending=(?P<pending>\d+)\s*$",
    re.MULTILINE,
)


@dataclass(frozen=True)
class DetectHealth:
    """The ``detect ...`` line of ``stats``: frames the detect task lost."""

    dropped: int
    stale: int
    notice_dropped: int
    shed: int
    stale_read: int


@dataclass(frozen=True)
class AngleQueueStatus:
    """The ``angles ...`` line of ``triggerLog perf``: the club's pending
    angles (l3_angle_queue.h)."""

    queued: int
    done: int
    stale: int
    failed: int
    dropped: int
    pending: int


@dataclass(frozen=True)
class AcceptanceCheck:
    name: str
    passed: bool
    detail: str


def parse_detect_health(text: str) -> DetectHealth | None:
    match = _HEALTH.search(text)
    if match is None:
        return None
    return DetectHealth(
        dropped=int(match.group("dropped")),
        stale=int(match.group("stale")),
        notice_dropped=int(match.group("notice")),
        shed=int(match.group("shed")),
        stale_read=int(match.group("stale_read")),
    )


def parse_angle_queue(text: str) -> AngleQueueStatus | None:
    match = _ANGLES.search(text)
    if match is None:
        return None
    return AngleQueueStatus(**{name: int(value) for name, value in match.groupdict().items()})


def evaluate_acceptance(
    *,
    verify: DetectCoreStatus,
    dss: DetectCoreStatus,
    timing: DetectTiming | None,
    stats_text: str,
    perf_text: str,
    recoveries: int = 0,
) -> list[AcceptanceCheck]:
    """Each acceptance check with its verdict. A line the board did not
    report (an older image) fails its check: nothing passes unseen.
    ``recoveries``: times the run found the board stopped and not latched
    (it scores nothing until restarted) and restarted it."""
    health = parse_detect_health(stats_text)
    angles = parse_angle_queue(perf_text)
    service = None if timing is None else timing.stats.get("service")
    return [
        AcceptanceCheck(
            "verify_scored", verify.verify > 0, f"{verify.verify} frames scored on both cores"
        ),
        AcceptanceCheck(
            "verify_mismatches",
            verify.mismatches == 0,
            f"{verify.mismatches} mismatches (first: {verify.first_mismatch})",
        ),
        AcceptanceCheck("verify_failures", verify.failures == 0, f"{verify.failures} DSS failures"),
        AcceptanceCheck("dss_scored", dss.dss > 0, f"{dss.dss} frames scored on the DSS"),
        AcceptanceCheck(
            "dss_fallbacks", dss.fallbacks == 0, f"{dss.fallbacks} fell back to the MSS"
        ),
        AcceptanceCheck(
            "dss_not_latched",
            not dss.latched and dss.active == "dss",
            f"active={dss.active} latched={dss.latched} latches={dss.latches}",
        ),
        AcceptanceCheck(
            "detect_dropped_stale",
            health is not None
            and health.dropped == 0
            and health.stale == 0
            and health.stale_read == 0,
            "no detect line in stats"
            if health is None
            else f"dropped={health.dropped} stale={health.stale} stale_read={health.stale_read}",
        ),
        AcceptanceCheck(
            "keeps_up",
            timing is not None and timing.keeps_up,
            "no timing"
            if timing is None
            else (
                f"service mean {service.mean if service else '?'} us of {timing.budget_us}, "
                f"{timing.margin_negative} slot overruns"
            ),
        ),
        AcceptanceCheck(
            "board_never_stopped",
            recoveries == 0,
            f"found stopped and restarted {recoveries} times",
        ),
        AcceptanceCheck(
            "angles_not_dropped",
            angles is not None and angles.dropped == 0,
            "no angles line in triggerLog perf"
            if angles is None
            else (
                f"queued={angles.queued} done={angles.done} stale={angles.stale} "
                f"failed={angles.failed} dropped={angles.dropped} pending={angles.pending}"
            ),
        ),
    ]


# The detect timing's statistics, in the order a frame lives them.
_TIMING_ORDER = ("wait", "score", "service", "latency", "arrival")


def _us(value: int | None) -> str:
    return "-" if value is None else f"{value} us"


def format_timing_report(timing: DetectTiming | None, label: str) -> list[str]:
    """``triggerLog timing`` as the acceptance run prints it: the two
    deadlines, every statistic (wait, score, service, latency, arrival, then
    any other the board reports), and the last frames' timelines."""
    if timing is None:
        return [f"timing {label}: no frame decided yet"]
    lines = [
        f"timing {label}: frames={timing.frames} budget={timing.budget_us} us "
        f"over_budget={timing.over_budget} depth_max={timing.depth_max} ring={timing.ring} "
        f"margin_min={_us(timing.margin_min_us)} margin_last={_us(timing.margin_last_us)} "
        f"margin_negative={timing.margin_negative}"
    ]
    names = [n for n in _TIMING_ORDER if n in timing.stats]
    names += sorted(n for n in timing.stats if n not in _TIMING_ORDER)
    if not names:
        lines.append("  (no statistics)")
    for name in names:
        stat = timing.stats[name]
        lines.append(
            f"  {name:<9} n={stat.count} min={stat.min} mean={stat.mean} "
            f"max={stat.max} last={stat.last} us"
        )
    if timing.timeline:
        lines.append("timeline (oldest first):")
        for event in timing.timeline:
            line = (
                f"    slot={event.slot} epoch={'post' if event.epoch is None else event.epoch} "
                f"core={event.core} wait={event.wait_us} "
                f"score={'-' if event.score_us is None else event.score_us} "
                f"service={event.service_us} latency={event.latency_us} depth={event.depth}"
            )
            if event.flags:
                line += " flags=" + ",".join(sorted(event.flags))
            lines.append(line)
    return lines
