1 Need to install some software dependencies:

	run : 
	
	sudo apt install ros-foxy-nmea-msgs
	
	pip install pyserial
	
	pip install transforms3d
	
	sudo apt install ros-foxy-tf-transformations
	
	sudo apt install libpcap0.8-dev
	
	sudo apt install libyaml-cpp-dev
	
	
	or run: ./soft_init.sh

2 for gnns:
  1) Create ros2 workspace: 
  
  	run : mkdir -r gnss_ws/src
  
  2) run : 
  	
  	cp gnss-D300 gnss_ws/src -rf
  
  3) build run : 
  
     cd gnss_ws
     
     colcon build
     
  4) run driver: 
     
     sudo chmod 777 /dev/ttyUSB*
     
     source install/setup.bash
     
     ros2 launch nmea_navsat_driver nmea_serial_driver.launch.py
 

3 for imu:
  1) Create ros2 workspace: 
  	
  	run : mkdir -r imu_ws/src
  
  2) run :
   
  	cp imu-MTI-G-710-6A8G4 imu_ws/src -rf
  
  3) build run : 
     
     cd imu_ws
     
     colcon build
     
  4) run driver: 
     
     sudo chmod 777 /dev/ttyUSB*
     
     source install/setup.bash
     
     ros2 launch xsens_driver xsens_driver.launch.xml
     
4 for lidar:
  1) Create ros2 workspace: 
  	
  	run : mkdir -r lidar_ws/src
  
  2) run : 
  
  	cp lidar-RoboSense-RS-Helios1615 lidar_ws/src -rf
  
  3) build run : 
     
     cd lidar_ws
     
     colcon build
     
  4) set ip:
     
     You need to set the IP of the network card connected to the lidar,The default setting is: 192.168.1.102
     
  5) run driver: 
     
     source install/setup.bash
     
     ros2 launch rslidar_sdk start.py

