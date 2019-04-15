# Yuneec Autopilot
This is a fork of the PX4 Done Autopilot
The original readme can be found [here](https://github.com/PX4/Firmware/blob/master/README.md)

## Yuneec build targets
| Drone  | make target  |
|---|---|
| H520  | `make yuneec_tap-v2_default`  |
| H520C  | `make yuneec_tap-v3_default`  |
| H520C (OFDM) | `make yuneec_tap-v3_ofdm`  |
| Mantis Q  | `make yuneec_tap-v4_v18s` |

## Creating new release branch
1. Update the develop branch on your machine: `git checkout develop && git pull`
2. Create new branch: `git checkout -b release-x.x.x`
3. Set the correct `RELEASE_VER` variable [tap_common/rcS](https://github.com/YUNEEC/Firmware/blob/develop/ROMFS/tap_common/init.d/rcS)
4. Add the branch to the CI white-list in [.travis.yml](https://github.com/YUNEEC/Firmware/blob/develop/.travis.yml#L10-L13)
  ```
  branches:
    only:
    - develop
    - /^v[0-9].*$/
  ```
