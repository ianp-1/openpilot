# US 2023 ID.4 gateway candidate

Based on infiniteCable2/openpilot `a6c755d8877d28ebedaf139061878ba4df823047`
and infiniteCable2/opendbc `dadd82517910737b429c531e7573fc08d4ee9411`.

## Vehicle and changes

The observed vehicle is a US 2023 ID.4, VIN chassis E8/year P, with radar
`1EA907572H / 0234`, camera `1EA980653D / 5332`, and a gateway harness.
Public tests use a synthetic VIN. Raw logs and vehicle/device identifiers are
kept private.

The custom firmware fallback recognizes this narrow VIN/firmware combination as
MK1. The camera is recorded as logging-only, so an explicit ECU allowlist admits
it to the custom matcher; generic exact/fuzzy matching still excludes logging
records. Missing, unknown or conflicting pair evidence is rejected by this
custom fallback. This does not override earlier generic matching paths.

The MK1 platform now collects CAN for at least five seconds after firmware
acquisition. In the saved capture, the former 1.679-second window missed gateway
bus1 signatures, while a 5.008-second window captured them. This adds startup time
to the shared MK1 ID.4/ID.5 platform. It is a minimum collection period, not a hard
wall-clock timeout when the receive callback blocks.

The observed firmware pair remains dashcam-only if gateway evidence is absent or
Motor_51 (0x10B) / ESC_51 (0xFC) lengths do not match MK1's 32/48 bytes. Topology is
never forced. The passive path uses noOutput and skips controls initialization.

MK1 gateway BSM presence detection now includes bus2, where the existing parser
consumes BSM. Existing bus0 evidence is retained so loss of an expected bus2
stream still causes a validation error. BSM and ACC use the existing bus2 receive
path; required message liveness, checksums and counters remain active. No Panda
safety source or control-message transmit behavior was changed.

Battery-panel telemetry remains optional for CAN liveness. Incomplete or stale
values hide the panel rather than displaying misleading values.

## Actual saved-log evidence

One 48.60-second raw log from the installed first candidate was copied and
replayed offline. The user reports that the car was started and in Park.

- Recorded first-candidate recognition was MOCK/dashcam-only: the logging-only
  camera had been discarded before the custom matcher.
- Updated automatic firmware matching plus extended CAN collection produces
  VOLKSWAGEN_ID4_MK1, gateway, BSM enabled and dashcamOnly=False.
- MK1 gateway receive replay: 4,546/4,546 updates valid after a three-second
  warmup; BSM and ACC_19 consumed on bus2 with no reported receive errors.
- Compiled MK1 Panda safety RX: all 21,485 required bus0 frames accepted, and
  complete RX configuration valid for 4,546/4,546 post-warmup samples.
- GEN2 complete safety RX configuration was invalid for all those samples:
  required Motor_51/ESC_51 frame lengths do not fit. A true return for an
  unmatched individual packet is not evidence that the configuration is valid.

This is parked receive evidence, not validation of moving blind-spot targets,
steering/braking output, or driving performance.

## Tests and limits

- 141 vehicle/parser/UI/CAN-fingerprint tests passed, with 3,749 subtests.
- 410 MEB safety tests passed, with 282 subtests and 124 inherited skips.
- Regression tests cover actual logging flags, invalid firmware context,
  delayed/missing gateway evidence, conflicting bus0 BSM/ACC copies, and loss
  of required bus2 BSM/ACC streams.
- Independent review found no must-fix findings.
- A broader firmware suite has 365 failures reproduced on the unmodified
  previous candidate as well; these were not introduced or repaired here.

The updated branch has not been built/run on the comma or evaluated driving.
Saved-log replay cannot establish road safety. The development work did not
install updates, reboot the device or change vehicle settings.

## Try while parked

Candidate installer: https://installer.comma.ai/ianp-1/id4-2023-gateway

Update the installed branch and allow the software to restart while parked with
the vehicle fully awake. Verify MK1 recognition, gateway location, stock
longitudinal control, valid CAN and no controls/safety mismatch. If unsupported
car, CAN errors or dashcam mode persist, save a new log for inspection; do not
force a model or bypass the guards. Do not enable Alpha Long for this test.

The main repository pins the matching public opendbc commit so the matcher,
vehicle parser, battery schema and UI remain compatible.
