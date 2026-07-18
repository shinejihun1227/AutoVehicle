# Current Environment State

Last updated: 2026-07-13

## 1. Host PC

- Host OS: Windows
- MORAI SIM execution: Windows host PC
- MORAI installation: installed
- MORAI license: competition license available
- Competition map/license target: K-City

## 2. ROS Development Environment

- Virtual machine: installed
- Guest OS: Ubuntu 20.04
- ROS version: ROS1 Noetic
- Current role: development and study environment

## 3. Important Competition Caveat

The current Ubuntu 20.04 + ROS Noetic setup is inside a VM. This is useful for study, early ROS package development, topic checks, and algorithm prototyping.

However, the competition rulebook states that desktop PC usage is required and VMware/VirtualBox usage support is not provided. Therefore, the final competition run should be prepared on a native Ubuntu 20.04 desktop environment unless the organizers explicitly allow the VM setup.

Project policy:

1. Use the VM for learning and early implementation.
2. Keep all scripts and packages portable.
3. Prepare a migration checklist for native Ubuntu 20.04 before final competition testing.
4. Avoid relying on VM-specific network behavior when designing MORAI communication.

## 4. Immediate Information Still Needed

- MORAI official example package location
- VM network mode: bridged, NAT, or host-only
- Windows host IP
- Ubuntu VM IP
- MORAI sensor setting files
- Competition-provided mission scenario files
- Competition judge program files
- Final target PC plan: VM only, dual boot, separate Ubuntu desktop, or lab desktop

## 5. Notion Workspace

- Connected Notion workspace: `염지훈`
- Main project hub: https://app.notion.com/p/39a043d83d5e810ca841ea98cf9da194

If the team uses a specific shared Notion page or teamspace, move or recreate the project hub under that shared parent page.

