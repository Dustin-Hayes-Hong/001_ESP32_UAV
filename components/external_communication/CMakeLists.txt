#只有在这里写出的文件会被编译器编译和被其它地方访问
#这个文件修改一次需要重新编译所有文件
#在这个文件里 # 号是注释

idf_component_register(
SRCS            #这里标记出目录下的c文件
"WIFI/WIFI.c"   "UDP/UDP.c"     "remote_control/remote_control.c"   "anotc_client_v4.34/anotc.c"
			
INCLUDE_DIRS  	#这里标记出目录中存放h文件的文件夹
"WIFI/include"  "UDP/include"   "remote_control/include"    "anotc_client_v4.34/include"

REQUIRES        #标记出需要的资源
esp_wifi
nvs_flash
inside_communication
flight_controller
VBAT

PRIV_REQUIRES   #标记出需要的资源


)