# ObstacleAvoidanceOutput message sent by Realsense CHT board to autopilot

uint32 flags 	# 0x01=camera OK
float32 vx		#obstacleAvoidanceSpeed output NED frame N m/s
float32 vy		#obstacleAvoidanceSpeed output NED frame E m/s
float32 vz		#obstacleAvoidanceSpeed output NED frame D m/s
float32 yawspeed 	# radians/s