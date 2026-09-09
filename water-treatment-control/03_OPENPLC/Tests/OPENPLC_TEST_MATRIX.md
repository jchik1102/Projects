# OpenPLC Test Matrix

| ID | Test | Stimulus | Expected result |
|---|---|---|---|
| P01 | Heartbeat watchdog | Increment HR9 every 0.1 s | C151 true; C176 false |
| P02 | Communication loss | Hold HR9 for more than 2 s | C176 true; C51-C57 false; state 900 if active |
| P03 | Automatic start | Pulse C104 then C101 | HR306=1; state 10/20; one P-101 command |
| P04 | Fill permissive | LIT101 <=5% | Both transfer commands off |
| P05 | Normal treatment | T-201 80%, concentration in band for 20 s | Dose -> Mix -> Verify |
| P06 | Batch transfer | Accepted batch, valve open feedback | P-201 command only after C7 true |
| P07 | Rejected batch protection | Quality invalid | No P-201 command |
| P08 | Pressure demand | PIT301 below SP | HR304 rises; lead booster commanded |
| P09 | Booster staging | Total demand >100% | Both boosters commanded when available |
| P10 | Lead failure | C251 true while A is lead | Lead changes to B and C55 asserted |
| P11 | Failed start | Command without run feedback for 2 s | Corresponding failed-start alarm |
| P12 | Manual interlock | Manual P-101 request with T-101 low-low | Request rejected |

Automated initial tests are implemented in `test_openplc_modbus_smoke.m` and `run_openplc_acceptance_tests.m`. Full plant-level validation must be repeated after the Simulink Modbus blocks are connected.
