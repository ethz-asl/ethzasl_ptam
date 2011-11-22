Header header      
float64               imuTime               # time since start of imu in us
float32[3]            acceleration          # calibrated acc sensor output in m/s^2
float32[3]            rotation              # rotational velocity in rad/s
float32[3]            angles                # angles derived by datafusion in rad
float32[3]            mag                   # no units, just direction?
float32               dHeight               # differential height, m/s
float32               height                # height , m
float32               batteryVoltage        # battery voltage in V
float64               latitude 
float64               longitude 
int16[4]              ctrlInput             # control inputs roll, pitch, yaw, thrust
int16                 motors                # 
int16                 status                # 