# Autopilot Releases Changelog
All notable changes to this project will be documented in this file.

## [Unreleased]
### Fixed
* H520 HITL functional even without battery plugged in (#3164)
* Improved corner case handling when ESC firmware update fails (#2907)
* "Wait for GPS or use indoor mode" mavlink message too long (#3296)
* Improved speech output of "Geofence violated" message (#3274)
* [PUBLIC] Reconnecting RC now stops RC alarm (#3297)
* Tunes are no longer interrupted by themselves when sent multiple times (#3297)
* Improved speech output of "Geofence violated" message (#3274)
* `free` command working again in nuttx shell. Have to mount `/proc` on startup. (#3302)
* [Regression] Fix unreliable RC binding on H520C (tap-v3) (#3304)

### Changed
* Re-ordered startup scripts into board specific files for tap-v1/2/3/4 (#3284)
* Commander logging strings improved readability and consistency (#3315)

### Added
* [PUBLIC] Update description for COM_RC_LOSS_PARAM (#3317)


### Added
* [Regression] Restored the "factory calibration by shaking the vehicle" feature (#3306) 


### Added
* Logging whether Firmware flashed was built using RESTRICTED_BUILD or not (#3318)


## [v1.8.0-1.6.5]
### Fixed
(needs to be cherry-picked)


## [v1.8.0-1.6.4]
### Changed
* Increase return to ground control station safety distance from 3m to 5m (#3196)

### Fixed
* Force landing gear to keep the state during takeoff (#3197)
* Gimbal yaw problems in ROI (#3204)
* Prevent old ROI settings from interfering with follow-up missions (#3203)


## [v1.8.0-1.6.3]
### Changed
* Increased maximum allowed distance to 1st mission waypoint from 900m to 4000m (#3182)

### Fixed
* [Regression] Restored functionality of structure scan. Bug introduced with 1.6.1 (#3187)


## [v1.8.0-1.6.2]
### Fixed
* Landing gear rise when doing compass calibration, corner case when toggling the switch during calibration(#3045, #3157)


## [v1.8.0-1.6.1]
### Added
* RC loss alarm enabled for when RC is disconnected after flying, but the drone is still powered (#3126)
* Keep vehicle heading constant during ROI. This improves yaw-tracking in ROI. (#3153)

### Fixed
* Pauses at the end of a tune are now respected. Fixes low battery and error tunes. (#3159)
* [Regression] Hotfix landing gear not lowering in RTL to GCS because altitude 0 (#3162, #3110)
* Fixed eccentric camera pitching while ROI is active (#3166)
* Lower case messages for battery warnings etc, fixing weird pronunciation in DataPilot (#3171)


## [v1.8.0-1.6.0]
### Changed
* Changed minimum altitude for OBS. avoidance from 1.5m to 2.0m (#2675)
* New (less annoying) tune for low-battery warning (#3095)
* Only rotate gimbal to face towards ROI, not the entire drone (#3121)


### Fixed
* The drone wouldn't face straight forward when flying a survey after a ROI mission (#3121)
* Don't run motor-check in HITL (#3120, #3089)
* Camera now has correct orientation in structure scan (#2322, #3121)
* [Regression] Play low-battery warning tunes only once (#3073)
* Gimbal-lock within acceptance radius (#3137)


### Added
* Added landing gear state to logger (#3111, #3117)
* Only allow takeoff without GPS in indoor mode (#2808, #3055)


## [v1.8.0-1.5.10]
### Fixed
* Setting COM_ARM_SWISBTN parameter to avoid problems with the arm button (#3085)

### Added
* Command motors to stop if they don't spin up correctly for safety (#3089)


## [v1.8.0-1.5.9]
### Fixed
* Brake at waypoint in mission if stop is required (#3076)


## [v1.8.0-1.5.8]
### Fixed
* Drone takes off when obstacle avoidance is activated after arming, even without throttle input (#3042)
* Drone won't disarm when landing with obstacle avoidance active (#3042)
* "Tuning" the gimbal feedforward scale for less offset in orbit (#3047)
* Prevent error tunes from being spammed while connecting to the drone (#3052)
* Slow landing detection problems (#3054)


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


## v1.7.3-1.4.22
### Fixed
* Fix jMAVSim SITL on macOS (#2899)
* Fix SITL packager for macOS (#2899)
* ST10C loiter button now fully interrupts safety RTL (#2964)


## v1.7.3-1.4.21
### Fixed
* Porting Preflightcheck fixes form upstream (#2911)

## v1.7.3-1.4.20
### Changed
* Fix tortoise slider not working after vehicle configuration reset (#2894)


## v1.7.3-1.4.19
### Changed
* Always resetting COM_RC_LOSS_MAN to 1 on reboot (#2829)

### Fixed
* AP not removing old logs from SD card when it fills up (#2844)
* Brief slow-downs in survey missions (#2849)
* "yaw-to-north" fix (again) for gimbal version 2.39.9 (#2839)


## v1.7.3-1.4.18
### Fixed
* Fix yawing to North for LOITER-TIME mission items (#2805)


## v1.7.3-1.4.17
### Fixed
* HITL not connecting to the drone, fixed using a known NuttX patch (#2793, #2813)


## v1.7.3-1.4.16
### Fixed
* Parameter reset happining if one version number different (from #2726)
* Fix parameter acks for int parameters (#2762)
* Added deadzone for RC gimbal control (#2700, #2708)


## v1.7.3-1.4.15
### Fixed
* Landing gear goes up if drone tilted (#2737)
* ST10C RTL doesn't persist after 1st button push (#2738)
* Properly initialize gimbal attitude (#2730)
* Use absolute yaw angle mode for gimbal (#2730)
* Fix gimbal yaw angle during RTL and land (#2730)
* Fix USB connection regression (#2730)


## v1.7.3-1.4.14
### Added
* Time estimate for RTL (#2560, #2076)
* Added safety trigger based on RTL time estimate (#2563)

### Changed
* Removing logger in HITL, frees up memory (#2713, #2493)


## v1.7.3-1.4.13
### Added
* Resetting parameters when switching between releases (#2691)


## v1.7.3-1.4.12
### Fixed
* Team-Mode pitch- and yaw-mode set by slave (#2699)


## v1.7.3-1.4.11
### Changed
* Lifting restriction on parameter `COM_RC_LOSS_MAN` (#2602)


## v1.7.3-1.4.10
### Added
* Added ST10c support (#2506)
* Added parameter for adjusting RTL cone radius (#2592)

### Fixed
* Lifting restriction on parameter `SYS_PARAM_VER` (#2589, #2575, #2579)
* Too small minimal yawspeed with turtle mode (#2251, #2594)


## v1.7.3-1.4.9
### Fixed
* Sonar-based obstacle avoidance increased trigger range and reliability (#2456)


## v1.7.3-1.4.8
### Fixed
* Switched to mixed channel RC channel parsing for team mode slave

### Added
* Use left stick for slave pitch gimbal control in velocity mode


## v1.7.3-1.4.7
### Fixed
* Fixed Five-Rotor-Mode issues (#2406)


## v1.7.3-1.4.6
### Fixed
* Remote control loss reaction when slave disconnects (#2381)


## v1.7.3-1.4.5
### Fixed
* Increase parameter default version number to apply new values for 1.5 beta testers


## v1.7.3-1.4.4
### Fixed
* Team mode reconnection problems (#2391)


## v1.7.3-1.4.3
### Added
* Basic Team mode support (#2349)


## v1.7.3-1.4.2
### Fixed
* Yaw input also gets scaled using the turtle slider (#2235)


## v1.7.3-1.4.1
### Fixed
* Multi-battery missions now resumable after low-battery emergency landing (#1999)


## v1.7.3-1.4.0beta2
### Added
* Yaw stick deadzone and exponential curve (#1910)

### Changed

### Fixed
* Compass calibration uses less memory, now also works after a flight (#1884)


## v1.7.3-1.4.0beta1
### Changed
* Allow changing RC mode in flight (#1840)

### Fixed
* Battery deep discharge when USB is plugged
* Low battery beep warning no longer continuous
* Land descend rate slow down depending on mode


## v1.7.3-1.4.0alpha1
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
* Direction change delay in rabit mode
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
