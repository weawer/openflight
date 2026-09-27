# Trigger approach-rate regression fix

- Binary: `l3_dump_2ms_iq16_trigger_gatefix_20260927.bin`
- SHA-256: `468fcec50fe5fb058edf5f41c89a515c9376ed6164c71122e78ec91c98792d35`
- TI native build passed with warnings as errors.
- Hardware status: this image has not been tested on the board.

The default multi-frame trigger could fire on a strong stationary return
inside the tee gate. Resetting the approach origin made elapsed time zero,
which bypassed the minimum approach-rate check while track age remained two.
The fix rejects this case when the configured minimum approach rate is
positive. Explicit single-observation and zero-rate configurations retain
their existing behavior. Native regressions reproduce both stationary and
backward-moving cases before the fix and pass afterward.

The matching smoke script saves failure stats and `triggerLog` before
stopping the sensor. It checks initial health too: an already-fired detector
or startup error must not be accepted as a clean baseline.

## Outstanding timing failure

`openflight_sessions/iwr-trigger-port-smoke/run.jsonl` reports 3,224 us maximum
combined frame work against a 2,000 us period, plus one missed HWA frame.
This fix does not resolve that workload problem. The same log reports two
detector observations before firing but contains no trigger frame records;
the precise cause of that hardware return is not established.

This is a diagnostic image, not a validated 2 ms release. Hold off on the
long soak or real-shot acceptance until combined processing timing is
addressed. Preserve IQ16 and the current frame period when doing that work;
do not mask the failure by raising trigger thresholds.
