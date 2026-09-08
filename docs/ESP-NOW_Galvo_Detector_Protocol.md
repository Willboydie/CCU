# ESP-NOW Galvo Detector Network — Engineering Operation Manual

**Revision:** 3.0
**Status:** Corrected system description and firmware-aligned operating details

---

## Contents

1. [Purpose and scope](#1-purpose-and-scope)
2. [System architecture](#2-system-architecture)
3. [Functional roles](#3-functional-roles)
4. [Power-up, enrolment and loss of communication](#4-power-up-enrolment-and-loss-of-communication)
5. [Operating state sequence](#5-operating-state-sequence)
6. [USB serial command interface](#6-usb-serial-command-interface)
7. [Firmware hill-climbing beam-centering algorithm](#7-firmware-hill-climbing-beam-centering-algorithm)
8. [Serial responses](#8-serial-responses)
9. [Firmware defaults and user-adjustable settings](#9-firmware-defaults-and-user-adjustable-settings)
10. [RSSI diagnostics](#10-rssi-diagnostics)
11. [Typical localisation observation workflow](#11-typical-localisation-observation-workflow)
12. [Implementation notes and limitations](#12-implementation-notes-and-limitations)
13. [Troubleshooting](#13-troubleshooting)
14. [Associated firmware files](#14-associated-firmware-files)

---

## 1. Purpose and scope

This manual describes the operating philosophy, communications protocol, serial interface and firmware behaviour of the ESP-NOW smart-tag network used by a galvo-controlled laser projector. The network comprises a Raspberry Pi or PC host, one ESP32-C3 master/gateway and up to `MAX_SLAVES` ESP32-C3 optical detector tags.

The host communicates with the master through USB serial. The master handles ESP-NOW enrolment, polling and tag selection. Each tag reports only its measured optical intensity and battery-monitor ADC value when requested.

---

## 2. System architecture

```
Raspberry Pi / PC host  ⇄  USB serial  ⇄  ESP32-C3 master  ⇄  ESP-NOW  ⇄  optical smart tags
```

- The master is the only ESP-NOW device directly controlled by the host.
- Each smart tag has a unique MAC address and a per-boot random nonce.
- A tag measures detector intensity and battery ADC level only when polled.
- The master polls sequentially, so only one request/reply exchange is outstanding at a time.

---

## 3. Functional roles

### 3.1 Host / projector controller

- Associates target identifiers with projector operations.
- Commands a network-wide `SCAN` to identify which tag is illuminated.
- Runs the search algorithm in angular coordinates to centre the laser on the tag.
- Issues repeated `SAMPLE` commands after each galvo perturbation and compares returned intensity values.
- Records the galvo encoder angles at the intensity maximum for subsequent localisation or registration calculations.

### 3.2 ESP32-C3 master/gateway

- Enrols up to `MAX_SLAVES` tags by source MAC address.
- Assigns displayed slave numbers 1 to `MAX_SLAVES` for the current master power cycle.
- Sends a one-second `ENROL_ACK` heartbeat to every enrolled tag.
- Performs sequential identification scans and selects the greatest returned intensity.
- Polls only the selected tag during optimisation.
- Reports detector, battery and received-signal data to the host.

### 3.3 Optical smart tag

- Broadcasts `HELLO` every 500 ms while unenrolled.
- Accepts enrolment only when the returned boot nonce matches its current boot.
- Flashes the external LED rapidly while searching and slowly while enrolled.
- Remains radio-silent except for `HELLO` broadcasts while searching and `SAMPLE_REPLY` packets when polled.
- Returns averaged raw ADC values for detector intensity and battery monitoring.
- Enters indefinite deep sleep after a user-defined period of remaining unenrolled.

---

## 4. Power-up, enrolment and loss of communication

1. The tag powers up, generates a new boot nonce and begins rapid LED flashes and `HELLO` broadcasts.
2. The master receives `HELLO`, stores or refreshes the tag record, adds the ESP-NOW peer and returns `ENROL_ACK` containing the same nonce.
3. The tag validates the nonce, stores the master MAC address, enters `LISTEN` mode and changes to the slow enrolled LED indication.
4. The master repeats `ENROL_ACK` every `HEARTBEAT_INTERVAL_MS` (default 1,000 ms), using it as a heartbeat.
5. If the tag receives no valid packet from its enrolled master for `MASTER_LOSS_TIMEOUT_MS` (default 5,000 ms), it removes the master peer, becomes unenrolled and resumes `HELLO` broadcasts.
6. If it then remains unenrolled for `SLEEP_TIMEOUT_SECONDS` (default 300 s), it enters indefinite deep sleep. No timer or GPIO wake source is configured; cycling the physical power switch is required to restart it.

> **Sleep timer interpretation:** The sleep countdown applies while the tag is unenrolled. A successful enrolment stops the countdown. Loss of an established master starts a fresh unenrolled countdown.

---

## 5. Operating state sequence

```
Power-up → Discovery → Enrolment → Idle/LISTEN → SCAN → Selected
         → repeated SAMPLE optimisation → target angle stored → NEXT/SCAN
```

Only one tag is selected at a time. Selection controls which tag the master polls during repeated `SAMPLE` operations. Other enrolled tags remain in `LISTEN` mode and are not sampled until the next `SCAN` or RSSI diagnostic.

The network does not autonomously move the galvo. Galvo motion and the optimisation strategy belong to the host/projector firmware. The master provides deterministic intensity measurements and tag identity.

---

## 6. USB serial command interface

Commands are case-insensitive. `SCAN`, `NEXT` and `SAMPLE` use comma-separated target arguments. `RSSI` with a slave number uses a space.

| Command | Description |
|---|---|
| `HELP` | Display the command summary. |
| `LIST` | List all enrolled tags, including displayed number, MAC, latest detector ADC, battery ADC, RSSI quality and selection state. |
| `SCAN,<target>` | Clear the current selection, poll all enrolled tags sequentially and select the tag returning the greatest detector intensity. |
| `NEXT,<target>` | Operationally the same identification scan as `SCAN`; used when proceeding to another intended target/tag. |
| `SAMPLE,<target>` | Poll only the currently selected tag. This is the measurement command used repeatedly by the host galvo-optimisation algorithm. |
| `RSSI` | Test every enrolled tag. Up to ten successful request/reply RSSI measurements are averaged for each tag. |
| `RSSI <slave>` | Run the same RSSI test for one displayed slave number; for example, `RSSI 2`. |
| `DESELECT` | Clear the current selected tag and return all enrolled tags to `LISTEN` mode. |

---

## 7. Firmware hill-climbing beam-centering algorithm

The network firmware supplies the intensity feedback needed by a separate host-side optimiser. The term "hill" refers to the intensity surface as a function of galvo angle, not to terrain or physical movement of tags.

```
I = f(θx, θy)
```

Here, `I` is the selected tag's returned detector intensity and `θx` and `θy` are the commanded galvo angles. Beam centring seeks the local maximum of this surface.

### 7.1 Recommended sequence

1. Aim the beam approximately towards the intended tag.
2. Issue `SCAN,<target>`. The master polls all tags and selects the one with the greatest returned intensity.
3. Request a baseline reading with `SAMPLE,<target>`.
4. Perturb one galvo axis by a small positive or negative angular step.
5. Issue another `SAMPLE,<target>` and compare the returned intensity with the previous value.
6. Retain a perturbation that increases intensity; reverse or reduce the step when intensity falls.
7. Repeat on both axes, reducing the step size as the maximum is approached.
8. When the stopping criterion is satisfied, record the galvo encoder angles as the centred observation for that tag.
9. Move towards the next intended tag and issue `NEXT,<target>` or `SCAN,<target>` to identify it before repeating the optimisation.

> **No threshold required for selection:** The current master chooses the greatest received intensity among responding tags. It does not apply an optical detection threshold. The host may nevertheless reject implausibly low or noisy maxima as an application-level validity check.

### 7.2 What SCAN and SAMPLE do not do

- `SCAN` does not centre the beam; it only identifies the strongest responding tag.
- `SAMPLE` does not alter the galvo; it returns one averaged measurement from the selected tag.
- The master does not currently calculate gradients, choose perturbation directions or determine convergence.
- Tags do not compare their own samples or autonomously hand over tracking.

---

## 8. Serial responses

| Response | Meaning |
|---|---|
| `ENROLLED,<slave>,<MAC>` | A new tag has been assigned a displayed slave number. |
| `READY,<count>` | Current number of enrolled tags. Emitted after enrolment and by `LIST`. |
| `SLAVE,<slave>,<MAC>,<intensity>,<battery>,<RSSI>,<quality>,<state>` | Per-tag `LIST` record. Detector and battery fields are the latest stored raw ADC values. |
| `READING,<target>,<sample>,<slave>,<intensity>,<battery>,<RSSI>` | One response obtained during an all-tag `SCAN`/`NEXT` operation. |
| `ACTIVE,<target>,<slave>,<intensity>,<battery>,<RSSI>` | The tag selected at the end of `SCAN`/`NEXT`. |
| `SAMPLE_RESULT,<target>,<sample>,<slave>,<intensity>,<battery>,<RSSI>` | A measurement from the currently selected tag. |
| `RSSI,<slave>,<mean>,<quality>` | Averaged RSSI result. |
| `RSSI,<slave>,NO_REPLY,POOR` | No replies were received during the RSSI attempts. |
| `RSSI_DONE,<count>` | RSSI diagnostic completed; count is the number of tag results reported. |
| `TIMEOUT,<target>,<sample>,<slave>` | No matching sample reply arrived within `SAMPLE_TIMEOUT_MS`, default 40 ms. |
| `DESELECTED` | The current tag selection was cleared. |
| `ERROR,<text>` | Command, enrolment or polling error. |

---

## 9. Firmware defaults and user-adjustable settings

| Setting | Default | Purpose |
|---|---|---|
| Master `SERIAL_BAUD` | 921600 baud | USB serial rate. |
| Master `SAMPLE_TIMEOUT_MS` | 40 ms | Maximum wait for a matching reply. |
| Master `HEARTBEAT_INTERVAL_MS` | 1,000 ms | `ENROL_ACK` heartbeat period. |
| Master `IDENTIFY_AVERAGE_COUNT` | 8 | ADC samples averaged during `SCAN`/`NEXT`. |
| Master `TRACK_AVERAGE_COUNT` | 8 | ADC samples averaged during `SAMPLE`. |
| Master `RSSI_PACKET_COUNT` | 10 | RSSI attempts per tested tag. |
| Slave `HELLO_INTERVAL_MS` | 500 ms | `HELLO` broadcast interval while unenrolled. |
| Slave `MASTER_LOSS_TIMEOUT_MS` | 5,000 ms | Time without valid master traffic before returning to discovery. |
| Slave `SLEEP_TIMEOUT_SECONDS` | 300 s | Maximum unenrolled period before indefinite deep sleep. |
| Slave `MAX_AVERAGE_COUNT` | 32 | Upper limit accepted for requested ADC averaging. |
| Slave `STATUS_LED_PIN` | GPIO 5 | User-selectable external status LED. |
| Slave `DETECTOR_ADC_PIN` | GPIO 0 | User-selectable detector-amplifier ADC input. |
| Slave `BATTERY_ADC_PIN` | GPIO 3 | LOLIN C3 Pico battery-monitor input; solder bridge must be closed. |

> **ADC interpretation:** Detector intensity and battery level are reported as raw 12-bit ADC values. The current firmware does not convert the battery reading to volts; conversion requires the actual board divider ratio and ADC calibration.

---

## 10. RSSI diagnostics

RSSI is a radio-link diagnostic and is not used to select the illuminated optical tag. For each tested tag, the master performs ten sample request/reply attempts with a random delay of 10 to 40 ms between attempts and averages the RSSI values of successful replies.

| Quality | Range |
|---|---|
| `EXCELLENT` | −50 dBm or stronger |
| `GOOD` | −51 to −65 dBm |
| `USABLE` | −66 to −75 dBm |
| `MARGINAL` | −76 to −85 dBm |
| `POOR` | below −85 dBm |

---

## 11. Typical localisation observation workflow

1. Power all required tags and the master. Confirm `READY` and use `LIST` to verify membership.
2. Optionally run `RSSI` to confirm reliable radio coverage. RSSI is not an optical alignment measurement.
3. Direct the galvo approximately towards the first reference tag and issue `SCAN,<target>`.
4. Use repeated galvo perturbations and `SAMPLE,<target>` measurements to maximise intensity.
5. Store the final galvo encoder angles and the selected tag identity.
6. Move the beam towards the next reference tag and issue `NEXT,<target>`.
7. Repeat until the required set of angular observations has been acquired.
8. Use the stored tag observations in the separate pose-recovery or coordinate-registration calculation.

---

## 12. Implementation notes and limitations

- The current compiled master description states a maximum of four tags. `MAX_SLAVES` remains a configuration limit rather than a fundamental ESP-NOW protocol limit, but increasing it changes scan duration and may require queue/timing review.
- Displayed slave numbers are 1-based; internal firmware array indices are 0-based.
- Slave assignments are stable only for the current master power cycle unless persistent storage is added.
- The master accepts the greatest returned intensity without an absolute detector threshold or explicit tie-break rule beyond first-greatest encountered order.
- A tag replies only to the master MAC address stored during valid enrolment.
- ESP-NOW encryption is currently disabled.
- The master stores the latest intensity, battery and RSSI values; `LIST` may therefore show stale measurements until another poll occurs.
- The `slaveMode` field records `LISTEN` or `SELECTED` status. The present slave still responds to a valid directed sample request in either mode; radio silence is enforced by the master's polling policy.
- The 40 ms sample timeout should be validated under the final RF environment and chosen ADC averaging count.

---

## 13. Troubleshooting

| Symptom | Check |
|---|---|
| Tag flashes rapidly continuously | It is unenrolled. Check that master and tag use the same ESP-NOW channel and protocol header, that the master has a free slot, and that the tag has not yet reached its sleep timeout. |
| Tag LED is off after being left unattended | The tag may be in indefinite deep sleep. Cycle its physical power switch off and on. |
| `MASTER_LOST` appears on tag serial output | No valid master packet was received within 5 s. The tag has returned to discovery and restarted its 300 s default sleep countdown. |
| `SCAN` reports no detector replied | Check enrolment, detector wiring, sample timeout, ESP-NOW channel and supply. The error means no valid `SAMPLE_REPLY` was received, not merely that laser intensity was low. |
| Wrong tag selected | Ensure the intended beam is not illuminating another tag more strongly. The master always selects the greatest returned raw intensity and applies no threshold. |
| `SAMPLE` says no active slave selected | Run `SCAN` or `NEXT` successfully before starting the optimisation loop. |
| Battery value appears incorrect | The value is a raw ADC code. Confirm the LOLIN C3 Pico battery-monitor solder bridge and apply the correct divider/calibration conversion in host software. |

---

## 14. Associated firmware files

This revision was checked against the following supplied sketches:

- `master_smart_tag_v4_0.ino` — ESP32-C3 master/gateway firmware.
- `slave_smart_tag_v4_0.ino` — ESP32-C3 optical smart-tag firmware, including indefinite deep sleep after the user-selected unenrolled timeout.

Both sketches require the shared header `GalvoDetectorProtocol.h` to be present in the same Arduino sketch folder or otherwise available on the compiler include path.
