## docker ##

docker stop $(docker ps -q)         # stop all running containers

docker rm $(docker ps -aq)          # remove all containers
docker rmi $(docker images -q)      # remove all images
docker system prune -a --volumes    # remove everything

docker ps                           # list active containers
docker ps -a                        # list all containers
docker images -a                    # list images
docker volume ls                    # list volumes
docker network ls                   # list networks
docker start maui_jazzy              # start container ros_jazzy
docker exec -it maui_jazzy /entrypoint.sh  # enter container

## groups and users ##
getent group i2c # get the numeric group ID (GID) of the i2c group


## devices ## 
udevadm info -a -p $(udevadm info -q path -n /dev/ttyAMA0) # check for mathcing tag for device (ex /dev/serial0) for udev rules
# more info on UDEV RULES at https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Udev%20Rules.html

sudo dmesg | grep tty   # check serial ports

sudo fuser /dev/ttyACM0 # check processes running on port 
sudo kill xxxx          # fill process with PID xxxx 

sudo minicom -b 9600 -o -D /dev/ttyAMA0     # show terminal output


## gpio ##
# gpio commands -> https://lloydrochester.com/post/hardware/libgpiod-intro-rpi/

gpioinfo # get info on all gpio pins used
gpiodetect


## i2c ##
i2cdetect -l            # check i2c adresses available
i2cdetect -y 1          # check i2c device port number for i2c-1: 
i2cset -y 1 0x08 0x01   # send 1 byte through i2c at port 0x08


## gps ##
sudo systemctl enable gpsd.socket # enable the socket (necessary only the first time)
sudo systemctl start gpsd.socket # start the socket
sudo systemctl status gpsd.socket # check gpds
cgps -s                             # verify incoming data from gps: 


## memory ##
baobab      # verify memory usage 

## cameras
rpicam-hello --camera 0 --qt-preview -t 0 # get preview feed of the camera through ssh 


## ROS - when in docker container ros-jazzy ##
# create cpp package in ros2
ros2 pkg create --build-type ament_cmake --license Apache-2.0 [name] --dependencies rclcpp [other dependencies]
ros2 pkg create --build-type ament_cmake --license Apache-2.0 pwm_controller --dependencies rclcpp std_msgs sensor_msgs geometry_msgs

# create py package in ros2
ros2 pkg create --build-type ament_python --license Apache-2.0 [name] --dependencies rclpy [other dependencies]
ros2 pkg create --build-type ament_python --license Apache-2.0 maui --dependencies rclpy 

# start a bag reading all topics 
ros2 bag record -o [name] -a

# start a bag reading only some topics
ros2 bag record -o [name] [topic1] [topici]
ros2 bag record -o heading_movements /imu/data /imu/mag_raw

# publish static tf
ros2 run tf2_ros static_transform_publisher [x y z y p r] [parent_link] [child_link]
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

# check arduino 
docker start ros_jazzy 
docker exec -it ros_jazzy /entrypoint.sh  # enter container
ros2 run arduino_comm ardu_i2c_wave_node
ros2 topic pub /command std_msgs/msg/Int32MultiArray '{data: [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,27500, 27500, 27500, 27500]}'
ros2 topic pub /command std_msgs/msg/Int32MultiArray '{data: [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, -1, -1, -1]}'
ros2 topic pub /command std_msgs/msg/Int32MultiArray '{data: [0,0,0,0,0,0,0,0,0,0,0,0,0, 0, 0, 0]}'
ros2 topic pub /button_state std_msgs/msg/Bool '{data: True}'