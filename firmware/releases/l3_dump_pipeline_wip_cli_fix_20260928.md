# WIP pipeline: CLI scheduling fix

- Image: `l3_dump_pipeline_wip_cli_fix_20260928.bin`
- SHA-256: `4f1903f2a45b7495b657fcaef1894a4b5366ffba63b156ebab8c30bb7b7b6a9e`
- Source: commit `190a22dcc0323632d2a677bd9c1640b2eff3e77c` plus the adjacent
  `l3_dump_pipeline_wip_cli_fix_20260928.patch`.
- Build: TI SDK 3.6.2 LTS, ARM 20.2.7 LTS, C6000 8.3.3,
  DSPLIB C674x 3.4.0.0, native `make -C firmware build-native`.
- DATA_RAM free: 23,251 bytes, above the 16,384-byte policy minimum.
- Hardware: not flashed or tested by the agent.

This preserves the WIP tracking behavior rather than including the newer
viewer/club-follow changes. The original WIP image is unchanged. The source
fix is also applied to the current branch.

Control and HWA rearm still outrank detection, which now outranks polled CLI
and notice writes. Performance counters reset on each sensor start. No
trigger thresholds, IQ precision, capture geometry or frame periods change.
The matching host soak now checks for stale reads after diagnostic output,
so a clean pre-output snapshot alone cannot yield PASS.

Three regression tests failed before the fixes. Relevant host/native suite:
1,597 passed, 27 skipped. Ruff and diff checks passed. TI MSS/DSS build and
packaging passed. Tests do not establish real-board timing behavior.

To reproduce the source, extract `firmware/` from the source commit into an
isolated directory and apply the adjacent patch with `patch -p1` at its root.
Build with an explicit release name; never overwrite `l3_dump_pipeline_wip.bin`.
On this workstation the installed tool overrides are:

```bash
make -C firmware build-native \
  TI_ROOT=/home/jamorro/ti \
  MMWAVE_SDK_INSTALL_PATH=/home/jamorro/ti/mmwave_sdk_03_06_02_00-LTS/packages \
  R4F_CODEGEN_INSTALL_PATH=/home/jamorro/ti/ti-cgt-arm_20.2.7.LTS/ti-cgt-arm_20.2.7.LTS \
  C674_CODEGEN_INSTALL_PATH=/home/jamorro/ti/ti-cgt-c6000_8.3.3 \
  C674x_DSPLIB_INSTALL_PATH=/home/jamorro/ti/dsplib_c674x_3_4_0_0 \
  XDC_INSTALL_PATH=/home/jamorro/ti/xdctools_3_61_00_16_core \
  BIOS_INSTALL_PATH=/home/jamorro/ti/bios_6_73_01_01/packages \
  XWR68XX_RADARSS_IMAGE_BIN=/home/jamorro/ti/mmwave_sdk_03_06_02_00-LTS/firmware/radarss/xwr6xxx_radarss_rprc.bin \
  RELEASE_NAME=l3_dump_pipeline_wip_cli_fix_20260928.bin
```

After flashing and resetting, sync the host changes and rerun the armed
1,000-frame soak from the handoff, then 50,000 frames if clean. Both pre- and
post-diagnostic stats must stay clean. Retain the JSONL. This tests armed
pre-trigger operation; real-shot/post-impact timing still needs separate
validation. Frame-based gates are not yet normalized between 3 ms and 2 ms.
