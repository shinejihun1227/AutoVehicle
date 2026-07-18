# UDP Bridge Plan

## 1. Confirmed Network Values

```text
Windows MORAI IP: 192.168.0.151
Ubuntu algorithm IP: 192.168.0.200

Camera UDP receive port: 1001
LiDAR UDP receive port: 2001
GPS UDP receive port: 3001
IMU UDP receive port: 4001
CollisionData UDP receive port: 907
Competition Vehicle Status UDP receive port: 909
Ego Ctrl Cmd MORAI receive port: 9093
Ego Ctrl Cmd local port: 9094
```

## 2. Architecture

```text
MORAI UDP
  -> yeoms_udp_bridge
  -> /localization/ego_pose
  -> /localization/ego_twist
  -> yeoms_control Stanley
  -> /control/ctrl_cmd
  -> yeoms_udp_bridge UDP CtrlCmd sender
  -> MORAI
```

## 3. Important Direction Rules

Sensor/status packets:

```text
MORAI Windows -> Ubuntu algorithm PC
```

For sensor packets, use the Ubuntu destination port. For example:

```text
192.168.0.151.3000 > 192.168.0.200.3001
```

In this case, the bridge must listen on `3001`, not `3000`.

Control command:

```text
Ubuntu algorithm PC -> MORAI Windows
```

## 4. What Must Be Verified Before Driving

- UDP packets arrive on ports 1000, 2000, 3000, 4000, 907, 909.
- Competition Vehicle Status packet byte layout is confirmed.
- Ego Ctrl Cmd packet byte layout is confirmed.
- Steering sign is verified at very low speed.
- `CtrlMode`, `gear`, and `longCmdType` match the competition rules.

## 5. Why Raw Mode Exists

MORAI UDP packet layouts can differ by simulator version and competition package. The bridge therefore starts with packet monitoring and raw logging. After the exact byte offsets are known, the adapter can publish `/localization/ego_pose` and `/localization/ego_twist` reliably.
