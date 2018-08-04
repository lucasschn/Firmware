include(configs/nuttx_tap-v3_default)
px4_nuttx_configure(HWCLASS m7 CONFIG nsh ROMFS y ROMFSROOT tap_common TAP_ESC H520)
