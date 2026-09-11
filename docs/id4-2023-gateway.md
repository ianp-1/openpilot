# US 2023 ID.4 gateway candidate

This branch is a targeted development candidate based on infiniteCable2/openpilot
`a6c755d8877d28ebedaf139061878ba4df823047` and infiniteCable2/opendbc
`dadd82517910737b429c531e7573fc08d4ee9411`.

## Scope

The observed vehicle is a US model-year 2023 ID.4 with E8 in VIN positions 7–8,
radar `1EA907572H / 0234`, camera `1EA980653D / 5332`, and a gateway harness.
The public tests use a synthetic VIN; no device logs, full vehicle VIN, device
identifiers, credentials, or ECU serial numbers are included.

VW's [2023 US VIN filing](https://vpic.nhtsa.dot.gov/mid/home/displayfile/891a4edc-09ce-4b2c-91c6-ed0c976a67a3)
lists E8 for the ID.4 and P for model year 2023. E8 alone is not evidence of the
later MEB CAN protocol. Upstream's custom fuzzy matcher selected MK2 for this
observed ECU/VIN combination.

The exception recognizes only US E8, year P, with that exact radar/camera pair.
It selects the existing MK1 implementation. Missing, unknown, or conflicting
pair responses are rejected by this custom fallback. Other firmware-matching
paths and other vehicles retain upstream behavior.

An additional interface guard requires `Motor_51` (0x10B) to be 32 bytes and
`ESC_51` (0xFC) to be 48 bytes for this ECU pair. Missing or incompatible lengths
leave the vehicle in dashcam mode. This is a prerequisite, not proof that the
layout, checksums, steering, or braking are correct.

The MK1 implementation already receives gateway blind-spot messages from bus 2.
No CAN bus rerouting, Panda safety limits, control-message timeouts, checksums,
or counter validation are changed by this patch.

Battery-panel inputs are display-only and may be missing or irregular. Their
liveness no longer determines whether vehicle-control CAN is valid. Required
control inputs retain their validation. The panel is hidden until all its
inputs are fresh (within five seconds) and voltage/capacity are positive; it is
hidden again when any input becomes stale. This conservative display policy may
hide the panel on cars with slow battery telemetry.

## Validation status

Completed locally:

- Reproduced the incorrect classification through the full firmware-matching
  entrypoint, then verified the corrected result and negative ECU cases.
- Reproduced invalid CAN with otherwise valid synthetic control messages and
  absent display telemetry; verified that the regression passes after the fix.
- Tested missing steering messages, checksum rejection, repeated-counter
  rejection, incompatible generation lengths, and battery freshness.
- Ran the Volkswagen vehicle tests, CAN parser tests, and MEB safety tests.
- Tested the battery UI validity gate and Cap'n Proto serialization together.

Final local results: 130 vehicle/parser/UI tests and 3,718 subtests passed;
410 MEB safety tests and 282 subtests passed (124 inherited skips). All 45
measured added executable lines were covered, including 100% of the new battery
helper. The broader measured existing code was 74% covered; this is not a claim
of whole-project coverage. The local test environment's dependency audit found
no known vulnerabilities. The full on-device application build was not run.

Not completed:

- The actual saved raw CAN log replay. The device became unreachable before
  those logs could be copied locally. Earlier read-only inspection supplied
  firmware, VIN-prefix, and CAN-message-count evidence only.
- An on-device build, parked validation of the installed candidate, or a
  controlled driving evaluation. The current comma installation was not changed.

This branch is **not road-validated** and is not a verified cure for every CAN
error reported on the vehicle. Synthetic messages and passing inherited safety
tests cannot establish real vehicle compatibility.

## Installer and next checks

Candidate installer: `https://installer.comma.ai/ianp-1/id4-2023-gateway`

Before using vehicle controls, retrieve the saved raw logs and compare both
generations' frame lengths and CRC parsing. Replay the complete car interface
with the corrected classification and check all required CAN inputs. After
installation, verify classification, gateway location, stock longitudinal
control, CAN validity, and absence of controls/safety mismatches while parked.
Do not force a vehicle selection, enable Alpha Long, or bypass a CAN error to
make this candidate engage. Retain a way to restore the previous installation.

The main repository pins the paired public opendbc fork so the new battery
validity field and UI/schema changes are installed together.
