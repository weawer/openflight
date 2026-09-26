"""Runtime boundary joining TI capture to the frozen LCMF estimator."""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field, replace
from typing import Callable

from openflight.iwr6843 import tracking
from openflight.iwr6843.calibration import DEFAULT_TEE_RANGE_M, Calibration
from openflight.iwr6843.club import (
    ClubPathResult,
    ClubWindowPolicy,
    club_gate_m,
    estimate_club_path,
)
from openflight.iwr6843.late_window import (
    LateWindowPlan,
    capture_late_window,
    net_gate_m,
    plan_late_window,
    planner_mode,
)
from openflight.iwr6843.lcmf import (
    LCMFResult,
    PreparedLCMFCapture,
    estimate_lcmf_v1,
    prepare_lcmf_capture,
)
from openflight.iwr6843.monitor import IWR6843Capture, IWR6843CaptureMonitor
from openflight.iwr6843.recovery import (
    RecoveryCandidate,
    RecoveryPrior,
    find_recovery_candidates,
    select_recovery_candidate,
)
from openflight.iwr6843.sparse import (
    PowerSummary,
    SparsePlan,
    expand_cells,
    noise_cells,
    track_cells,
)

logger = logging.getLogger(__name__)

# The ball estimate's measured tdm_sign_used takes priority; this only
# resolves the TDM sign for the club-path fallback when it is unavailable.
# "auto" has no fixed sign of its own, so it defaults to positive, same as
# this module's own tdm_sign_policy default.
_TDM_SIGN_BY_POLICY = {"positive": 1, "negative": -1, "auto": 1}

# TrackMan holdout tracks centered almost exactly on OPS speed. A larger
# disagreement is unusual enough to justify a bounded alternate-track pass,
# but not enough by itself to choose an angle.
OPS_TRACK_SPEED_TOLERANCE_FRAC = 0.15
OPS_GUIDED_MAX_CANDIDATES = 8
OPS_GUIDED_MIN_LAUNCH_DEG = 2.0


def _ops_candidate_rank(candidate: RecoveryCandidate) -> tuple[float, int, float]:
    """Rank truth-free range walks before the more expensive LCMF pass."""
    return (
        abs(candidate.speed_ratio - 1.0),
        -candidate.track.n_inliers,
        candidate.track.rms_bins,
    )


def _credible_ops_candidates(
    candidates: list[RecoveryCandidate],
) -> list[RecoveryCandidate]:
    """Return a small, deduplicated set of OPS-compatible range walks."""
    credible = [
        candidate
        for candidate in candidates
        if abs(candidate.speed_ratio - 1.0) <= OPS_TRACK_SPEED_TOLERANCE_FRAC
        and candidate.track.n_inliers >= 12
        and candidate.track.rms_bins <= 0.48
        and candidate.track.t_last - candidate.track.t_first >= 0.009
    ]
    credible.sort(key=_ops_candidate_rank)
    selected: list[RecoveryCandidate] = []
    seen: set[tuple[int, int]] = set()
    for candidate in credible:
        # RANSAC emits many nearly identical lines. Keep one representative
        # per approximately 1 mph / 1.5 ms speed-impact cell.
        key = (
            round(candidate.track.speed_mph),
            round(candidate.impact_s / 0.0015),
        )
        if key in seen:
            continue
        seen.add(key)
        selected.append(candidate)
        if len(selected) >= OPS_GUIDED_MAX_CANDIDATES:
            break
    return selected


def _recovery_result_rank(
    candidate: RecoveryCandidate,
    result: LCMFResult,
) -> tuple[bool, float, float, int, float]:
    """Combine OPS agreement with independent spatial-estimator evidence."""
    # A single channel can still be useful (shot 6 on 2026-08-14), but must
    # beat a corroborated candidate by a meaningful OPS-speed margin.
    single_channel_penalty = 0.03 if result.single_channel else 0.0
    spread = float(result.component_std_deg or 0.0)
    return (
        result.single_channel,
        abs(candidate.speed_ratio - 1.0) + single_channel_penalty + 0.01 * min(spread, 8.0),
        candidate.track.rms_bins,
        -result.n_frames,
        -candidate.track.n_inliers,
    )


@dataclass(frozen=True)
class IWR6843ShotResult:
    """Capture transport result and optional angle measurement."""

    capture: IWR6843Capture | None
    measurement: LCMFResult | None
    club_path: ClubPathResult | None = None


@dataclass
class IWR6843Runtime:
    """Configured TI hardware and estimator state for the server."""

    capture_monitor: IWR6843CaptureMonitor
    calibration: Calibration
    net_range_m: float | None
    flight_mode: str = "net"
    tx_order: str = "normal"
    capture_timeout_s: float = 12.0
    azimuth_offset_deg: float = 0.0
    horizontal_phase_reference_rad: float | None = None
    tdm_sign_policy: str = "positive"
    club_window_policy: ClubWindowPolicy = field(default_factory=ClubWindowPolicy)
    # The ball-derived impact anchor runs ~2 ms late: on the 2026-08-07
    # 55-shot session, the club track's tee-contact error minimized at -2 ms
    # (0.026 m vs 0.035 m uncorrected), independently matching the camera's
    # ball-departure timing. Applied to the club estimators only; the ball
    # pipeline keeps its own anchor.
    club_impact_correction_s: float = -0.002
    # Accepted ball tracks establish a truth-free rolling prior. A rejected
    # vertical solution may use that prior to recover impact timing for the
    # independent experimental club search, never to publish vertical launch.
    recovery_observations: list[tuple[float, float, float]] = field(default_factory=list)

    @property
    def self_trigger_enabled(self) -> bool:
        """True when the firmware trigger, not the sound gate, starts shots."""
        return self.capture_monitor.watch_self_trigger

    @property
    def tracking_net_m(self) -> float | None:
        """Net clamp for ball tracks. Open flight does not apply one."""
        return net_gate_m(self.flight_mode, self.net_range_m)

    def plan_late_window(
        self,
        *,
        ball_speed_mph: float,
        launch_angle_deg: float | None,
        spin_rpm: float | None,
    ) -> LateWindowPlan | None:
        """Late looks for an open-flight shot, or None in a net or without a launch."""
        mode = planner_mode(self.flight_mode)
        if mode is None or launch_angle_deg is None:
            return None
        plan = plan_late_window(
            mode,
            ball_speed_mph,
            launch_angle_deg,
            spin_rpm or 0.0,
            self.calibration.tee_range_m or DEFAULT_TEE_RANGE_M,
        )
        return plan if plan.enabled else None

    def measure_late_window(
        self,
        plan: LateWindowPlan,
        *,
        impact_timestamp: float,
        on_measured: Callable[[dict | None], None],
    ) -> bool:
        """Queue the late looks on the capture worker, which owns the radar.

        ``on_measured`` runs on that worker with the measurement, or None when
        the look was missed or failed. Returns False when nothing was queued.
        """
        monitor = self.capture_monitor
        tee_range_m = self.calibration.tee_range_m or DEFAULT_TEE_RANGE_M

        def measure(_radar) -> None:
            result: dict | None = None
            try:

                def capture(radar) -> None:
                    nonlocal result
                    result = capture_late_window(
                        radar,
                        plan,
                        impact_timestamp=impact_timestamp,
                        tee_range_m=tee_range_m,
                        restore_cfg=str(monitor.config_path),
                        now=time.time,
                        sleep=time.sleep,
                    )

                monitor.run_on_other_profile(capture)
                if result is not None:
                    logger.info("[IWR6843] Late window timing: %s", result.get("timing_s"))
            except Exception:  # pylint: disable=broad-exception-caught
                logger.warning("[IWR6843] Late-window measurement failed", exc_info=True)
                result = None
            on_measured(result)

        return monitor.submit("late-window", measure)

    def _remember_recovery_observation(
        self, measurement: LCMFResult, ball_speed_mph: float
    ) -> None:
        if (
            not getattr(measurement, "accepted", False)
            or getattr(measurement, "impact_t_s", None) is None
            or getattr(measurement, "track_speed_mph", None) is None
            or getattr(measurement, "track_span_s", None) is None
        ):
            return
        self.recovery_observations.append(
            (
                float(measurement.track_speed_mph) / ball_speed_mph,
                float(measurement.impact_t_s),
                float(measurement.track_span_s),
            )
        )
        del self.recovery_observations[:-50]

    def _recover_impact_time(
        self, raw: bytes, calibration: Calibration, ball_speed_mph: float
    ) -> float | None:
        if len(self.recovery_observations) < 3:
            return None
        ratios, impacts, spans = zip(*self.recovery_observations)
        try:
            prior = RecoveryPrior.fit(list(ratios), list(impacts), list(spans))
            candidates = find_recovery_candidates(
                raw,
                calibration,
                ball_speed_mph=ball_speed_mph,
                net_range_m=self.tracking_net_m,
            )
        except ValueError:
            # Older/raw-ADC firmware formats cannot run the snapshot recovery.
            # Preserve the original no-impact behavior rather than losing the shot.
            return None
        candidate = select_recovery_candidate(candidates, prior)
        return candidate.impact_s if candidate is not None else None

    def _ops_guided_measurement(  # pylint: disable=too-many-return-statements
        self,
        raw: bytes,
        calibration: Calibration,
        *,
        ball_speed_mph: float,
        club: str | None,
        baseline: LCMFResult,
        prepared: PreparedLCMFCapture,
    ) -> LCMFResult:
        """Replace a suspicious TI range walk with an OPS-compatible one."""
        speed = baseline.track_speed_mph
        if baseline.accepted and speed is None:
            return replace(baseline, status="accepted_track_speed_warning")
        speed_error = (
            abs(speed / ball_speed_mph - 1.0)
            if speed is not None and ball_speed_mph > 0.0
            else float("inf")
        )
        if baseline.accepted and speed_error <= OPS_TRACK_SPEED_TOLERANCE_FRAC:
            return baseline

        try:
            candidates = _credible_ops_candidates(
                find_recovery_candidates(
                    raw,
                    calibration,
                    ball_speed_mph=ball_speed_mph,
                    net_range_m=self.tracking_net_m,
                    prepared=prepared.vertical,
                )
            )
        except Exception as error:  # pylint: disable=broad-exception-caught
            logger.warning("[IWR6843] OPS-guided track search failed: %s", error)
            if baseline.accepted:
                return replace(baseline, status="accepted_track_speed_warning")
            return baseline
        recoveries: list[tuple[RecoveryCandidate, LCMFResult]] = []
        for candidate in candidates:
            result = estimate_lcmf_v1(
                raw,
                calibration,
                ball_speed_mph=ball_speed_mph,
                club=club,
                net_range_m=self.tracking_net_m,
                tx_order=self.tx_order,
                tdm_sign_policy=self.tdm_sign_policy,
                horizontal_phase_reference_rad=self.horizontal_phase_reference_rad,
                track_override=candidate.track,
                track_override_scope=candidate.scope,
                prepared=prepared,
            )
            if (
                result.accepted
                and result.n_frames >= 4
                and result.angle_deg is not None
                and result.angle_deg >= OPS_GUIDED_MIN_LAUNCH_DEG
            ):
                recoveries.append((candidate, result))

        if recoveries:
            _candidate, selected = min(
                recoveries,
                key=lambda item: _recovery_result_rank(item[0], item[1]),
            )
            status = (
                "accepted_ops_guided_single_channel"
                if selected.single_channel
                else "accepted_ops_guided"
            )
            return replace(selected, status=status)
        if baseline.accepted:
            return replace(baseline, status="accepted_track_speed_warning")
        return baseline

    def _ball_max_range_m(self) -> float | None:
        """Ball-gate clamp just short of the net, or None in open flight."""
        net = self.tracking_net_m
        return (net - 0.25) if net else None

    def _club_gate_m(self) -> tuple[float, float] | None:
        """Range gate around the tee where the club approaches, or None."""
        from openflight.iwr6843.club import (  # pylint: disable=import-outside-toplevel
            CLUB_APPROACH_DEPTH_M,
            CLUB_GATE_TEE_MARGIN_M,
        )

        tee = self.calibration.tee_range_m
        if not tee:
            return None
        return (max(0.35, tee - CLUB_APPROACH_DEPTH_M), tee + CLUB_GATE_TEE_MARGIN_M)

    def plan_sparse_cells(self, summary: PowerSummary) -> SparsePlan:
        """Name the range cells LCMF, club path, and the noise floor need.

        Ball-track centers come first, then their neighbors, then club-gate
        peaks, so a trimmed request drops club context before the ball.
        """
        geometry = summary.geometry
        track = tracking.find_ball_from_power(
            summary.power,
            geometry,
            max_range_m=tracking.track_max_range_m(self.tracking_net_m),
        )
        cells = track_cells(track, geometry) if track is not None else []
        tee = self.calibration.tee_range_m
        if tee:
            rows, bins = tracking.detection_peaks(
                summary.power, geometry, gates_m=(club_gate_m(tee),)
            )
            seen = set(cells)
            peaks = [(int(row) // summary.n_loops, absolute) for row, absolute in zip(rows, bins)]
            cells.extend(cell for cell in expand_cells(peaks, geometry) if cell not in seen)
        return SparsePlan(cells=tuple(cells), noise_cells=tuple(noise_cells(summary, cells)))

    def track_config_command(self) -> str:
        """``trackCfg`` line that gives the firmware tracker this rig's limits.

        Fields, all SI: loop period (s), range bin size (m), ball max range
        (m), club gate low and high (m). A zero max range disables the net
        clamp; a zero-width gate disables the club cells. 17 significant
        digits let the firmware's strtod land on the host's exact doubles.
        """
        from openflight.iwr6843.monitor import (  # pylint: disable=import-outside-toplevel
            read_capture_config,
        )
        from openflight.iwr6843.sparse import (  # pylint: disable=import-outside-toplevel
            RANGE_FFT_SIZE,
        )
        from openflight.iwr6843.tracking import (  # pylint: disable=import-outside-toplevel
            LOOP_PRI_S,
            RANGE_SPAN_M,
        )

        max_range = self._ball_max_range_m() or 0.0
        club_lo, club_hi = self._club_gate_m() or (0.0, 0.0)
        monitor_config = getattr(self.capture_monitor, "config_path", None)
        config_path = str(monitor_config) if monitor_config is not None else None
        loop_period_s = LOOP_PRI_S
        if config_path is not None:
            configured_period = read_capture_config(config_path).loop_period_s
            if configured_period is not None:
                loop_period_s = configured_period
        fields = (
            loop_period_s,
            RANGE_SPAN_M / RANGE_FFT_SIZE,
            max_range,
            club_lo,
            club_hi,
        )
        return "trackCfg " + " ".join(f"{value:.17g}" for value in fields)

    def process_shot(  # pylint: disable=too-many-arguments
        self,
        *,
        impact_timestamp: float | None,
        ball_speed_mph: float,
        club: str | None,
        club_speed_mph: float | None = None,
        tilt_deg: float | None = None,
    ) -> IWR6843ShotResult:
        """Match one OPS shot to TI data and run LCMF-v1."""
        capture = self.capture_monitor.capture_for_shot(
            impact_timestamp,
            timeout_s=self.capture_timeout_s,
        )
        if capture is None or not capture.valid or capture.raw is None:
            return IWR6843ShotResult(capture=capture, measurement=None)
        shot_calibration = self.calibration
        if tilt_deg is not None:
            shot_calibration = replace(self.calibration, tilt_rad=math.radians(tilt_deg))
        prepared = prepare_lcmf_capture(capture.raw)
        if capture.noise_power is not None:
            # A sparse cube is mostly zeros; its own median would be ~0.
            prepared.vertical.set_noise_power(capture.noise_power)
        measurement = estimate_lcmf_v1(
            capture.raw,
            shot_calibration,
            ball_speed_mph=ball_speed_mph,
            club=club,
            net_range_m=self.tracking_net_m,
            tx_order=self.tx_order,
            tdm_sign_policy=self.tdm_sign_policy,
            horizontal_phase_reference_rad=self.horizontal_phase_reference_rad,
            prepared=prepared,
        )
        if isinstance(measurement, LCMFResult):
            measurement = self._ops_guided_measurement(
                capture.raw,
                shot_calibration,
                ball_speed_mph=ball_speed_mph,
                club=club,
                baseline=measurement,
                prepared=prepared,
            )
        horizontal_deg = getattr(measurement, "horizontal_deg", None)
        if horizontal_deg is not None:
            measurement = replace(
                measurement,
                horizontal_deg=horizontal_deg + self.azimuth_offset_deg,
                horizontal_raw_deg=horizontal_deg,
            )
        self._remember_recovery_observation(measurement, ball_speed_mph)
        club_path = None
        # No OPS club speed means no identity gate to distinguish the club
        # track from hands, body, or the ball itself, so an estimate here
        # would be an unverifiable guess -- worse than no estimate at all.
        if club_speed_mph:
            ball_sign = getattr(measurement, "tdm_sign_used", None)
            fallback = ball_sign not in (-1, 1)
            policy_sign = _TDM_SIGN_BY_POLICY.get(self.tdm_sign_policy, 1)
            impact_t_s = getattr(measurement, "impact_t_s", None)
            recovered_impact = False
            if impact_t_s is None:
                impact_t_s = self._recover_impact_time(
                    capture.raw,
                    shot_calibration,
                    ball_speed_mph,
                )
                recovered_impact = impact_t_s is not None
            if impact_t_s is not None:
                impact_t_s += self.club_impact_correction_s
            club_path = estimate_club_path(
                capture.raw,
                shot_calibration,
                ops_club_speed_mph=club_speed_mph,
                # Where impact sits in the ring, from the ball's own range
                # walk. The freeze is requested by a UART command, so the
                # trigger frame lands late by a variable 2-4 frames and the
                # slot cannot be assumed; None means the ball tracker gave
                # nothing to anchor to and club path declines.
                impact_t_s=impact_t_s,
                aim_offset_deg=self.azimuth_offset_deg,
                phase_reference_rad=self.horizontal_phase_reference_rad,
                tdm_sign=policy_sign if fallback else ball_sign,
                window_policy=self.club_window_policy,
            )
            if recovered_impact:
                club_path.status = f"{club_path.status}_recovered_impact"
            if fallback:
                # The ball measurement had no usable sign, so this is the
                # configured policy's guess, not a measured value. Recorded
                # in the status so a later replay can tell the two apart.
                club_path.status = f"{club_path.status}_tdm_sign_fallback"
        return IWR6843ShotResult(capture=capture, measurement=measurement, club_path=club_path)

    def stop(self) -> None:
        """Release TI hardware."""
        self.capture_monitor.stop()


__all__ = ["IWR6843Runtime", "IWR6843ShotResult"]
