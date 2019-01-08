# Autopilot Releases Changelog
All notable changes to this project will be documented in this file.

## [Unreleased]
### Fixed
* Drone takes off when obstacle avoidance is activated after arming, even without throttle input (#3042)
* Drone won't disarm when landing with obstacle avoidance active (#3042)
* "Tuning" the gimbal feedforward scale for less offset in orbit (#3047)


## [v1.8.0-1.5.7]
### Changed
* Increased maximum distance between waypoints from 900m to 4000m (#2978)

### Fixed
* Vehicle gets stuck during mission (#2998)
* Regression fix: hickup when stopping in slowed down descend (#2997)


## [v1.8.0-1.5.6]
### Fixed
* Orbit commands exceeding maximum radius (#2968)
* Fixed altitude lock during mission (#2960)
* Regression: fix slow landing (#2959)
* Regression fix: in mission, lock position when position setpoint reached waypoint (#2951)

### Added
* Orbit telemetry for UI (#2906)
* vehicle_local_position_setpoint logging improvment (#2959)


## [v1.8.0-1.5.5]
### Fixed
* Parameter update within FlightTask (#2931)
* Keep landing gear up during mission (#2938)
* ST10C loiter button now fully interrupts safety RTL (#2964)
* Fixed tap_esc motor feedback ID mapping (#2958)
* Fixed correcting motor not spinning in FTC (#2958)


## [v1.8.0-1.5.4]
### Changed
* Set SYS_RELEASE parameter to 15 for release branch 1.5.0

### Fixed
* Startup tune chopped up when updating drone (#2923)


## [v1.8.0-1.5.3]
### Added
* Arming without propellers now still spins 5 out of 6 motors (#2463)
* More detailed logging of motor faults (#2463)

### Changed

### Fixed
* Fixed scenario where H520 can perform full flips (#2701, #2645)
* Vehicle wobbles around yaw axis for big setpoint steps (#2896)
* Fixed Five-Rotor-Mode when more than one motor fails (#2463)


## [v1.8.0-1.5.2]
### Added
* Added parameter `RTL_CONE_DIST` to change size of the RTL cone (#2595, #2559)
* Resetting parameters when switching between releases (#2691, #2692)


## [v1.8.0-1.5.1]
### Added
* RTL to ground control station (#1879)

### Changed
* Lifting restriction on parameter `SYS_PARAM_VER`.


## [v1.8.0-1.5.0]
### Added
* ESC firmware version logging (#1786)
* ROI: New waypoint types for Region of Interest (missing in DataPilot) (#1819)
* Structure scan (#1819)
* Yaw stick deadzone and exponential curve (#1910)

### Changed
* Yaw P gain lowered from 6.0 to 3.5 because of upstream merge (#1872)
* Obstacle Avoidance Interface (#1827)
* Sonar-based obstacle avoidance increased trigger range and reliability (#1959)

### Fixed
* Compass calibration uses less memory, now also works after a flight (#1884)
* Connecting to drone after startup is now significantly faster (#2049)
* Multi-battery missions now resumable after low-battery emergency landing (#1999)


## [v1.7.3-1.4]
See ongoing changelog here: https://github.com/YUNEEC/Firmware/blob/release_1.4.0/CHANGELOG.md


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
