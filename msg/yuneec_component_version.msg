# version information for yuneec camera/gimbal
uint64 timestamp	 # time since system start (microseconds)

char[32] model_name	 # E90, E50 etc. Not supported by gimbal
uint32 firmware_version  # For camera: v << 24 & 0xff = Country, v << 16 & 0xff = Patch, v << 8 & 0xff = Minor, v & 0xff = Major
                         # For gimbal: v << 24 & 0xff = Major,   v << 16 & 0xff = Minor, v << 8 & 0xff = Patch, v & 0xff unused

# TOPICS yuneec_component_version yuneec_camera_version yuneec_gimbal_version
