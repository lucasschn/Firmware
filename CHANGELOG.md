# Autopilot Releases Changelog

## [Unreleased]
### Added
* HITL support for H520

### Changed
* Improved crash detection (#1546)

### Removed

### Fixed
* ecl: Update to version that fixes GPS noise bug (#1603)
* mavlink: don't send wrong time (#1578)
* navigator: don't continue cam triggering automatically after 2nd takeoff (#1563)

## v1.6.5-1.3.1 - 
*   Fix battery monitoring, add multiple checks for current measurement handling to prevent error cases.
*   Fix Barometer device ID, factory calibration gets recognized again.

## v1.6.5-1.3.0 - 2018-02-28
### Added
* Allow critical battery action `RTL` and emergency battery action `LAND` to be
  interrupted by mode switch.
* Faster subsequent param download (volatile param improvement).
* Parameter for enabling/disabling Fault-Tolerant-Control: `FTC_ENABLE`
* Add support for H520C
* Add support to Flight Tasks
* Add support to RealSense, only for H520C
* Enable internal dialect for MAVLink, custom Yuneec messages
* Add support to switch LED off

### Changed
* Battery estimation adjustments for slightly longer flight time.
* Use simple param to set LEDs on or off.
* Allow LED mode parameter to be changed.
* ESC firmware update `1.19 -> 2.0`
* Improved robustness on the estimator preflight checks (avoid flyaway)
* Refactoring of the `tap_esc` driver
* ESC firmware functions moved from `tap_esc` to `tap_esc_config`
* Increased landing speed from 0.8 m/s to 1.0 m/s

### Removed

### Fixed
* Don't continue triggering in a second mission.
* Send `SET_CAMERA_MODE` to component ID of camera.
* prevent vehicle getting stuck on low battery when a user switches out of the failsafe.
* Stop image and video capture and distance triggering on `RTL` but
  continue when a mission is paused.
* Fix for race condition in gimbal control.
* Correct LED colors during landing and after interrupted `RTL`.
* Restored former stability of FTC (sacrifice yaw control for roll and pitch)
* Correct mode switch at start up
* Sonar no longer causing wiggle when flying upwards in front of obstacle
* Fix binding problem
* Mission upload improvements
* Hotfix for ESC update procedure that was resulting in unit bricked
* RC regained will exit `RTL`
