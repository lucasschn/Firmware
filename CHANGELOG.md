# Autopilot Releases Changelog
All notable changes to this project will be documented in this file.

## [Unreleased]
### Added
* Time estiamte for RTL (#2560, #2076)
* RTL to ground control station (#1879)
* ESC firmware version logging (#1786)
* ROI: New waypoint types for Region of Interest (missing in DataPilot) (#1819)
* Structure scan (#1819)
* Resetting parameters when switching between releases (#2691, #2692, #2693)

### Changed
* Yaw P gain lowered from 6.0 to 3.5 because of upstream merge (#1872)
* Obstacle Avoidance Interface (#1827)
* Sonar-based obstacle avoidance increased trigger range and reliability (#1959)

### Fixed
* Connecting to drone after startup is now significantly faster (#2049)
* Multi-battery missions now resumable after low-battery emergency landing (#1999)
* Compass calibration uses less memory, now also works after a flight (#1884)
* Fixed scenario where H520 can perform full flips (#2701, #2645)


## [v1.7.3-1.4.9]
### Fixed
* Sonar-based obstacle avoidance increased trigger range and reliability (#2456)


## [v1.7.3-1.4.8]
### Fixed
* Switched to mixed channel RC channel parsing for team mode slave
### Added
* Use left stick for slave pitch gimbal control in velocity mode


## [v1.7.3-1.4.7]
### Fixed
* Fixed Five-Rotor-Mode issues (#2406)


## [v1.7.3-1.4.6]
### Fixed
* Remote control loss reaction when slave disconnects (#2381)


## [v1.7.3-1.4.5]
### Fixed
* Increase parameter default version number to apply new values for 1.5 beta testers


## [v1.7.3-1.4.4]
### Fixed
* Team mode reconnection problems (#2391)


## [v1.7.3-1.4.3]
### Added
* Basic Team mode support (#2349)


## [v1.7.3-1.4.2]
### Fixed
* Yaw input also gets scaled using the turtle slider (#2235)


## [v1.7.3-1.4.1]
### Fixed
* Multi-battery missions now resumable after low-battery emergency landing (#1999)


## [v1.7.3-1.4.0beta2]
### Added
* Yaw stick deadzone and exponential curve (#1910)

### Changed

### Fixed
* Compass calibration uses less memory, now also works after a flight (#1884)


## [v1.7.3-1.4.0beta1]
### Changed
* Allow changing RC mode in flight (#1840)

### Fixed
* Battery deep discharge when USB is plugged
* Low battery beep warning no longer continuous
* Land descend rate slow down depending on mode


## [v1.7.3-1.4.0alpha1]
### Added
* Support for remote control modes 1,2,3,4 (#985)
* Remaining battery time estimate during flight
* Indoor mode to disable GPS usage completely (#1677)
* HITL (Hardware In The Loop) support for H520
* Adjustable rise and descend speeds (#1824)

### Changed
* New H520 ESC Firmware 2.03
* Switch to raw ST16S hardware input mapping (requires ST16 update!)
* Improved crash detection (#1546)

### Fixed
* Hover and capture producing blurry images (#1450)
* Direction change delay in rabbit mode
* For reported jerky flight
* Improved tune playback (#1771, #1883)


## [v1.6.5-1.3.1]
### Fixed
* Fix battery monitoring, add multiple checks for current measurement handling to prevent error cases.
* Fix Barometer device ID, factory calibration gets recognized again.
* navigator: don't continue cam triggering automatically after 2nd takeoff (#1563)


## [v1.6.5-1.3.0] - 2018-02-28
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
* Improved estimator robustness (with upstram merge)

### Changed
* Battery estimation adjustments for slightly longer flight time.
* Use simple param to set LEDs on or off.
* Allow LED mode parameter to be changed.
* ESC firmware update `1.19 -> 2.0`
* Improved robustness on the estimator preflight checks (avoid flyaway)
* Refactoring of the `tap_esc` driver
* ESC firmware functions moved from `tap_esc` to `tap_esc_config`
* Increased landing speed from 0.8 m/s to 1.0 m/s

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
