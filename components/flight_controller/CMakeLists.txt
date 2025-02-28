#只有在这里写出的文件会被编译器编译和被其它地方访问
#这个文件修改一次需要重新编译所有文件
#在这个文件里 # 号是注释

idf_component_register(
SRCS            #这里标记出目录下的c文件
"PWM/PWM.c"     "IMU/IMU.c"     "PID/PID.c"     "flight_control/flight_control.c"   "Data_declaration/Data_declaration.c"
			
INCLUDE_DIRS  	#这里标记出目录中存放h文件的文件夹 
"PWM/include"   "IMU/include"   "PID/include"   "flight_control/include"            "Data_declaration/include"

REQUIRES        #标记出需要的资源
esp_driver_ledc
esp_driver_gpio
inside_communication
sensor
external_communication

PRIV_REQUIRES   #标记出需要的资源

)