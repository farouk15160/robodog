# RS06 manufacturer datasheet audit

Checked 2026-09-28 against the manufacturer files below. The September specification changes several assumptions in the original simulation: **200 × 200 mm motor heatsink**, **8 N·m continuous stall torque**, **0.23 Ω line resistance**, and **0.012 kg·m² equivalent output inertia**. An 11 N·m rotating rating is not an 11 N·m continuous holding rating. These ratings do not establish the cooling available in the robot.

## Sources and version control

- **S**: [RobStride RS series specification, 2026-09-17](https://github.com/RobStride/Product_Information/blob/main/%E7%81%B5%E8%B6%B3%E6%97%B6%E4%BB%A3RS%E7%B3%BB%E5%88%97%E4%BA%A7%E5%93%81%E8%A7%84%E6%A0%BC%E4%BB%8B%E7%BB%8D%282026.09.17%29.pdf). RS06 occupies PDF pages 30–33, counting the cover as page 1. Tables and drawings were visually checked after extraction. SHA-256: `76c85c3c11f15bf3adc1676d6a4b8c931ffd9c18cec41221f8ea5a8b54e22ea1`.
- **M**: [RS06 English user manual 260713](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS06/RS06User%20Manual260713.pdf). References below use printed page numbers; add one for PDF page number. SHA-256: `f2790b989d7aa3f1ecee321b0ca773db18f07a7b1da647b38cb714d361d32b2f`.
- [Manufacturer repository README](https://github.com/RobStride/Product_Information) still describes July specifications and links a July overview filename, although its root contains the September overview. Use S for the newest published performance specifications, and retain M for protocol detail and explicit historical differences.

## Verified specifications

All motor output torque, speed and equivalent inertia values already include the internal 9:1 gearbox. The external knee belt is additional.

| Quantity | Manufacturer value | Source |
|---|---:|---|
| Motor mass | 621 g; July tolerance ±3 g | S p31; M p11 |
| Internal reduction | 9:1 | S p31 |
| Rotating rated torque | 11 N·m at 100 rpm, with 200 × 200 mm aluminum heatsink | S p31 |
| Continuous stall torque | 8 N·m | S p33 |
| Peak output torque | 36 N·m | S p31 |
| Rated supply | 48 V DC | S p31 |
| Supply range | 15–60 V DC | S p31 |
| No-load speed | 480 rpm ±10%, approximately 50.27 rad/s | S p31; rad/s calculated |
| No-load current | 0.98 Arms ±10% | S p31 |
| Rated phase-current amplitude | 14.3 Apk ±10% | S p31 |
| Peak phase-current amplitude | 57 Apk ±10% | S p31 |
| Torque constant | 1.1 N·m/Arms; July manual says 1.09 | S p31; M p9 |
| Line resistance | 0.23 Ω ±10% | S p31 |
| Inductance | 0.165 mH ±10%; line/phase convention not stated in table | S p31 |
| Equivalent inertia at low-speed output | 0.012 kg·m² | S p31, Chinese label explicitly specifies low-speed equivalent |
| Back EMF | 7.6 Vrms/krpm ±10%; avoid treating this as an output-shaft constant without clarification | S p31 |
| Operating environment | −20 to +50 °C | S p31 |
| Storage environment | −30 to +70 °C | S p31 |
| Magnetic encoders | 2 | S p31 |
| Pole count / drive | 28 poles / three-phase FOC | S p31 |

The drawing distinguishes an approximately 88 mm mounting envelope, 82 ±0.2 mm dimension, 70 mm rear can dimension, and 49 ±0.5 mm total axial length. The 33 ±0.5 mm dimension is not total motor depth. A single advertised diameter is insufficient to validate mounting clearance. [S p30; M p8.](https://github.com/RobStride/Product_Information/blob/main/%E7%81%B5%E8%B6%B3%E6%97%B6%E4%BB%A3RS%E7%B3%BB%E5%88%97%E4%BA%A7%E5%93%81%E8%A7%84%E6%A0%BC%E4%BB%8B%E7%BB%8D%282026.09.17%29.pdf)

## Current interpretation

The 14.3 A and 57 A values are **phase waveform peak amplitudes**, not battery/DC-bus current, and not RMS current. Assuming a sinusoidal phase waveform, dividing by √2 gives approximately 10.11 Arms rated and 40.31 Arms peak. Those are engineering conversions of S p31, not additional manufacturer specifications; a stationary current vector is not the same time-varying waveform.

The quoted torque constant gives an approximate low-load conversion from output torque to phase RMS current. At rated load, 11 / 1.1 = 10 Arms is consistent with the published rated-current amplitude and tolerance. At peak load, 36 / 1.1 = 32.73 Arms does **not** equal the manufacturer's 40.31 Arms equivalent peak value. Do not use a single linear torque constant as proof of peak current demand, or erase the discrepancy by silently replacing the peak-current specification. The documents reviewed do not provide enough detail to attribute the difference quantitatively to saturation or losses. [S p31.](https://github.com/RobStride/Product_Information/blob/main/%E7%81%B5%E8%B6%B3%E6%97%B6%E4%BB%A3RS%E7%B3%BB%E5%88%97%E4%BA%A7%E5%93%81%E8%A7%84%E6%A0%BC%E4%BB%8B%E7%BB%8D%282026.09.17%29.pdf)

M p49 gives `iq_ref` and `iqf` ranges of −57 to +57 A. This establishes protocol range, but does not explicitly define the Park-transform normalization or an `Iq`-to-phase-RMS conversion. Hardware `Iq` should remain labelled as motor Iq in A until its convention is confirmed; the √2 phase-waveform calculation above is not sufficient evidence to relabel hardware Iq as RMS.

Battery current must be measured on the DC bus or calculated using a documented electrical power model. Summing twelve phase-current readings is not a battery-current estimate. Similarly, using half the specified line resistance, 0.115 Ω, in a three-phase model is an **equivalent-wye modeling assumption**, not an independently specified phase-winding resistance. Resistance variation with winding temperature, switching/iron losses and gearbox losses remain relevant.

## Published speed and thermal envelopes

These are tabulated manufacturer points, not digitized curve estimates. The first table is the **48 V torque–speed capability**, not a continuous thermal rating. [S p32.](https://github.com/RobStride/Product_Information/blob/main/%E7%81%B5%E8%B6%B3%E6%97%B6%E4%BB%A3RS%E7%B3%BB%E5%88%97%E4%BA%A7%E5%93%81%E8%A7%84%E6%A0%BC%E4%BB%8B%E7%BB%8D%282026.09.17%29.pdf)

| Output torque, N·m | Output speed, rpm |
|---:|---:|
| 5 | 430 |
| 11 | 426 |
| 20 | 390 |
| 30 | 320 |
| 36 | 280 |

The thermal-equilibrium curve explicitly requires a 200 × 200 mm aluminum heatsink. Its table does not justify extrapolating the rotating curve to zero speed. [S p32.](https://github.com/RobStride/Product_Information/blob/main/%E7%81%B5%E8%B6%B3%E6%97%B6%E4%BB%A3RS%E7%B3%BB%E5%88%97%E4%BA%A7%E5%93%81%E8%A7%84%E6%A0%BC%E4%BB%8B%E7%BB%8D%282026.09.17%29.pdf)

| Output speed, rpm | Thermal-equilibrium torque, N·m |
|---:|---:|
| 100 | 11.2 |
| 160 | 11 |
| 220 | 10.5 |
| 280 | 9.87 |
| 340 | 9.4 |
| 400 | 9.3 |

Maximum overload times differ substantially between rotating and stalled operation. The rotating heading specifies the 200 × 200 mm heatsink; the stall table on the same page does not restate all test conditions. Neither table supplies a complete cooling/recovery model for repeated gait cycles, so the following times are not permission for indefinitely repeated overload pulses. [S p33.](https://github.com/RobStride/Product_Information/blob/main/%E7%81%B5%E8%B6%B3%E6%97%B6%E4%BB%A3RS%E7%B3%BB%E5%88%97%E4%BA%A7%E5%93%81%E8%A7%84%E6%A0%BC%E4%BB%8B%E7%BB%8D%282026.09.17%29.pdf)

| Torque, N·m | Rotating limit | Stall limit |
|---:|---:|---:|
| 8 | — | Rated |
| 11 | Rated | 200 s |
| 17 | 200 s | 15 s |
| 20 | 36 s | Not tabulated in S |
| 25 | 18 s | 5 s |
| 30 | 8 s | 1 s |
| 36 | 4 s | 1 s |

The older M p10 rotating table gives 5 s at 36 N·m, while S gives 4 s. M also states a 130 × 160 mm rated-load heatsink on p8 and 130 × 155 mm overload-test heatsink on p9; S increases the published rotating test plate to 200 × 200 mm. Keep these versions separate. Do not combine old overload points with new cooling conditions.

## Temperature interpretation

The −20 to +50 °C range is an operating-environment specification, not the winding shutdown threshold. M p10 describes per-phase calculated winding temperature using phase current and a motor thermistor, with uneven heating when stalled. It describes a 145 °C winding constraint and contains an ambiguous parenthetical reference to 180 °C actual temperature. That wording does not justify commanding operation up to 180 °C. Board, motor thermistor and calculated winding channels must remain distinct; protocol fault thresholds must be checked against installed firmware and register definitions. [M pp8–10.](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS06/RS06User%20Manual260713.pdf)

Robot warning/derating/shutdown values of 70/85/100 °C are project policy, not these manufacturer environment limits or proof that the simulation temperature corresponds to a real sensor. The simulation's lumped thermal resistance and heat capacity are not provided by S or M and remain uncalibrated.

The manual itself contains incompatible default overtemperature numbers: private-protocol type 21 lists **135 °C** for both fault and warning (M p46), whereas the general fault register explanation says motor-thermistor **145 °C** (M p30), and MIT fault feedback also says **145 °C** (M p77). Do not select one as an independently verified universal firmware threshold. Confirm the installed firmware and readable protection parameters; do not modify factory protection thresholds to make a simulation agree.

The register list distinguishes MCU internal temperature (`0x3005`), motor NTC (`0x3006`, M p23), and board temperature (`0x301f`, M p25), with temperature values scaled by ten. Private-protocol type 2 calls its feedback field only temperature (M p42), while MIT feedback explicitly calls its temperature winding temperature (M p75). A decoder using the private protocol cannot infer its sensor identity solely from the MIT description. [M pp23, 25, 30, 42, 46, 75, 77.](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS06/RS06User%20Manual260713.pdf)

## Configuration audit checklist

This table records the **baseline inspected before this audit's corrections**, in `ros2_ws/src/robodog_description/config/robstride06.yaml`. It preserves why changes were required; consult the live file for the implemented values.

| Field or interpretation | Baseline | Audit status / required treatment |
|---|---|---|
| Source revision | July manual only | Add September specification and preserve version conflicts |
| Mass / internal ratio | 0.621 kg / 9 | Verified |
| Envelope / depth | 0.088 m / 0.049 m | Consistent envelope assumption; inspect full drawing for fits |
| Rated / minimum / maximum voltage | 48 / 15 / 60 V | Verified |
| Supply voltage | 44.4 V | Robot assumption: two 6S packs in series, not a motor specification |
| Rotating rated / peak torque | 11 / 36 N·m | Values verified; rotating heatsink condition must become 200 × 200 mm |
| Continuous stall torque | Absent | Record 8 N·m separately; holding comparison against 11 N·m is optimistic |
| Rated / peak phase-current amplitude | 14.3 / 57 A | Verified; retain Apk labels |
| Rated / peak phase RMS current | 10.1116 / 40.3051 A | Derived sinusoidal equivalents; not bus current or verified Iq conversion |
| Torque constant | 1.09 N·m/Arms | July value; September value 1.1 |
| Line / phase resistance | Estimated 0.30 / 0.15 Ω | Replace line with published 0.23 Ω; 0.115 Ω phase equivalent remains derived |
| Output equivalent inertia / armature | Estimated 0.00972 kg·m² | Replace with published 0.012 kg·m²; knee becomes 0.048 kg·m² for ratio 2 |
| Bare rotor inertia | Estimated 0.00012 kg·m² | Not directly published; output inertia divided by 9² is only an equivalent inference |
| Peak torque duration | Assumed 2 s | Not conservative against 36 N·m stall limit of 1 s; preserve separate overload tables |
| Maximum operational speed | 20 rad/s | Robot policy, not no-load speed or rated operating point |
| Thermal resistance / capacitance | 1.31 K/W / 310.5 J/K | Uncalibrated assumptions; updated electrical data alone do not calibrate these |
| Temperature thresholds | 70 / 85 / 100 °C | Robot policy; distinguish sensor channel and firmware protections |
| Damping / friction / belt efficiency | 0.05 / 0.12 / 95% | Robot model assumptions, not manufacturer measurements |

## Consequences for this robot

At the assumed external belt ratio 2 and efficiency 0.95, the knee receives approximately **20.9 N·m rotating rated**, **15.2 N·m continuous stall**, and **68.4 N·m peak** from one RS06. Knee speed is half actuator output speed and the actuator's output equivalent inertia reflects by 2². These are transmission calculations from the cited motor ratings, not belt-strength certification or evidence of equivalent cooling.

The initial 19.719 kg simulation used the rotating 11 N·m reference and an estimated inertia. The corrected configuration now uses the published output inertia, electrical data and an 8 N·m operational baseline, with the 11 N·m rotating rating preserved separately. The [regenerated feasibility report](rs06_feasibility.md) records the exact evaluated configuration and per-joint loads. Its temperature model remains uncalibrated. Actual mounting heat transfer, phase/DC-current measurements, thermal telemetry and repeated overload recovery remain necessary to confirm sustained hardware performance.

The CAN integration now stops on a motor-reported overtemperature fault even if the numeric temperature channel is lower than the software cutoff. RS06 type-21 fault frames also trigger a latched protective stop. Because M p46 does not establish byte order for the fault/warning words, the decoder retains raw bytes: any nonzero fault word produces a generic protective communication flag, while warnings remain separate. It does not invent a specific cause from an unverified byte order. Normal feedback and zero-fault frames cannot silently clear that latch.

An explicit operator clear keeps the controller idle and motors disabled. The first request sends the motor clear command; clearing the software latch then requires a later request with healthy feedback received after that command. Cached reads cannot satisfy this requirement. No physical motor was accessed during this audit.
