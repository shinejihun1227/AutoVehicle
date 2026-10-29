# ji_ujin_merge
Based on new_final_ws 9b6b31fcc4f47628174a29bc04b1d4a11fd63d65.
Only ji_ujin_merge is authorised for this integration. Preserve new_final_ws,
final_ws, jihun_ws, merge_ws, morai_ws_full and test/roundabout.
Retain Competition-only state input, original bridge/ports, GPS/IMU/Competition
EKF, main route, controller PID and final signal overlay from new_final_ws.
Integrate upstream V6.1 decision modules only, excluding validation, trial driver,
standalone control publishers and external packet implementations.
Only the mapped roundabout region may impose mission speed/stop requests.
Tunnel camera lane priority is activated by the current EKF blackout status;
require actual stable observed lane boundaries and retain freshness watchdogs.
Read competition_reference baseline in the parent workspace and the integration
provenance and execution guide before modifying runtime contracts.
