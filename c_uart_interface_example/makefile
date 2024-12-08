all: git_submodule mavlink_control

mavlink_control: mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp
	g++ -g -Wall -I mavlink/include/mavlink/v2.0 mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o mavlink_control -lpthread

git_submodule:
	git submodule update --init --recursive

mavlink_test: test.cpp serial_port.cpp
	g++ -I mavlink/include/mavlink/v2.0 test.cpp serial_port.cpp -o test -pthread `pkg-config --libs --cflags opencv4` 

track_test : t.cpp
	g++ t.cpp -o t -pthread `pkg-config --libs --cflags opencv4` -lopencv_tracking

gimbal_test: gimbal_tracking.cpp serial_port.cpp
	g++ -I mavlink/include/mavlink/v2.0 gimbal_tracking.cpp serial_port.cpp -o gimbal_control -lpthread `pkg-config --libs --cflags opencv4` -lopencv_tracking

sdk_test: using_sdk_control.cpp
	g++ using_sdk_control.cpp -o using_sdk_control `pkg-config --libs --cflags opencv4` 

sdk_tracking: tracking_sdk.cpp
	g++ tracking_sdk.cpp -o tracking_sdk `pkg-config --libs --cflags opencv4` 

gimbal_position_control: gimbal_position_control.cpp
	g++ -I mavlink/include/mavlink/v2.0 serial_port.cpp gimbal_position_control.cpp -o gimbal_position_control `pkg-config --libs --cflags opencv4` 	

clean:
	 rm -rf *o mavlink_control
