# new_final_ws

Current user-authorised follow-up (2026-10-30): modify test/roundabout only for
the permitted junction entry/dropout lifecycle and exclusive >0.6 signal
confidence floor. This supersedes the older new_final_ws-only scope below
for that request. Preserve the other branches and this branch's roundabout,
Competition, localization and speed configuration. See
morai_ws/docs/20261030_SIGNAL_ENTRY_RELEASE_KO.md and
SIGNAL_ENTRY_RELEASE_TEST_RESULTS.json for the change and verification scope.

Based on origin/final_ws c97840c987880736b0b4b10ef346f90e904d9d28.
The catkin workspace is morai_ws/, unlike the morai_ws_full branch.
Read morai_ws/docs/NEW_FINAL_COMPETITION_KO.md and NEW_FINAL_PROVENANCE.json.

Preserve final_ws, jihun_ws, merge_ws and morai_ws_full. Only new_final_ws is
authorised for this migration. Preserve user settings and historical documents.
Vehicle status input is Competition-only on Ubuntu UDP 9081 by default.
Its reserved xyz position/acceleration and y/z velocity fields are unavailable.
Internal EgoVehicleStatus is a compatibility message, not a simulator input.
Localization remains GPS/IMU EKF; do not substitute status placeholders for pose.
Do not introduce the later highway/avoidance stack. The user authorised the
2026-10-29 camera/signal/right-turn/lane follow-up on new_final_ws; see
morai_ws/docs/20261029_CAMERA_SIGNAL_KO.md for the changed driving settings.
The rules baseline is v1.2/final 14 checkpoints; synthetic packet tests do not
establish live simulator or organiser-forwarder compatibility.

Run python3 morai_ws/tools/validate_new_final.py for offline verification.
