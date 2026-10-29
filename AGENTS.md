# new_final_ws

Based on origin/final_ws c97840c987880736b0b4b10ef346f90e904d9d28.
The catkin workspace is morai_ws/, unlike the morai_ws_full branch.
Read morai_ws/docs/NEW_FINAL_COMPETITION_KO.md and NEW_FINAL_PROVENANCE.json.

Preserve final_ws, jihun_ws, merge_ws and morai_ws_full. Only new_final_ws is
authorised for this migration. Preserve user settings and historical documents.
Vehicle status input is Competition-only on Ubuntu UDP 9081 by default.
Its reserved xyz position/acceleration and y/z velocity fields are unavailable.
Internal EgoVehicleStatus is a compatibility message, not a simulator input.
Localization initializes from GPS and uses IMU plus Competition forward speed in
profile 2, including GPS blackout. Do not substitute status placeholders for pose.
The user authorised the 2026-10-30 green-priority right turn and tunnel follow-up.
Do not introduce the later highway/avoidance stack. The user authorised the
2026-10-29 camera/signal/right-turn/lane follow-up on new_final_ws; see
morai_ws/docs/20261029_CAMERA_SIGNAL_KO.md for the changed driving settings.
The rules baseline is v1.2/final 14 checkpoints; synthetic packet tests do not
establish live simulator or organiser-forwarder compatibility.

Run python3 morai_ws/tools/validate_new_final.py for offline verification.
