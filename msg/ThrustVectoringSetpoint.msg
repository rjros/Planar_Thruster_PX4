uint64 timestamp  #time since system start (microseconds)
uint8      attitude_mode # Attitude mode sent by onboard pc
float32[6] servo_angle # values for servo motors
float32[3] force #normalized values for 3d thrust
float32[3] torque #normalized values for 3d torque
