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

## Version

**USB vendor-id**: [0x2EA9](https://www.the-sz.com/products/usbid/index.php?v=0x2EA9]&p=&n=)

**USB product-id**:

| USB product-id  |  board        |
| ------------- |:-------------:|
| 0x0483 | tap-v1 |
| 0x0010      | tap-v2 |
| 0x0032    | tap-v3  |

Defined in the `defconfig`-file of the board.

**board-id**

| board-id  |  board        |
| ------------- |:-------------:|
| 10 | tap-v1 |
| 66 | tap-v2 |
| 96 | tap-v3 |

TODO: where are these numbers come from? Why do they have to be different than `USB product_id`?

**Vehicle Products**

| Label | product-id | board |
| ------------- |:-------------:| :-------------:|
| H520  | 0x0010 | tap-v2
| H520-2 | 0x0032 | tap-v3
| Mobilicom  | 0x0033  | tap-v3

The rule for creating a product-id:
- products defined in `default.cmake`:  `
```
product-id = USB product-id
```

- products defined in `non-default.cmake`:
```
product-id = (USB product-id) + (number of additional products)
```