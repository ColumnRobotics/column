What I did to setup the Minnowboard Max - Cole

Downloaded mavros
* sudo apt-get ros-indigo-mavros
* sudo apt-get ros-indigo-mavros-extras

Created a catkin workspace
* mkdir -p ~/catkin_ws/src
* cd ~/catkin_ws/src
* catkin_init_workspace
* cd ..
* catkin_make

Created an el.bash file
* cd ~/catkin_ws
* vim el.bash
* -> I copied the one found in odroid
* -> I am not sure if it is necessary or not
* -> It may also need to be changed

Clone the april_tag repository
* cd ~/catkin_ws/src
* sudo apt-get install git
* git clone https://github.com/ColumnRobotics/april_tag.git
* cd ~/catkin_ws
* catkin_make
* -> Build successfull

Clone the column repository
* cd ~/catkin_ws/src
* git clone https://github.com/ColumnRobotics/column.git
* cd ~/catkin_ws
* catkin_make
* -> Build Successfull

Added the camera.yml file to .ros folder
* cd ~/.ros
* mkdir camera_info
* cd ~/catkin_ws/src/column
* cp pseye.yml ~/.ros/camera_info
* cd ~/.ros/camera_info
* mv pseye.yml camera.yml

Get gscam
* In odroid: tar czf gscam.tar.gz gscam
* scp odroid@192.168.0.120:~/Development/ROS-semi-official/src/gscam.tar.gz .
* tar -xzf gscam.tar.gz
* scp odroid@192.168.0.120:~/Development/ROS-semi-official/src/gscam_config.sh .
* -> Have not checked the settings for gscam yet -> need to setup hardware on minnowboard first
* sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
* cd ~/catkin_ws
* catkin_make 
* -> Build successfull

Get global_planner.bash
* mkdir -p ~/Development/scripts
* cd ~/Development/scripts
* scp odroid@192.168.0.120:~/Development/scripts/global_planner.bash .

Test gscam
* source ~/catkin_ws/src/gscam_config.sh
* in another tab: roscore
* rosrun gscam gscam
* -> catin_ws is sourced in ~/.bashrc
* Worked whenever I connected the camera to the USB port
* May not work with the other camera attached -> may need to check which video port it is

Test UART
* First test was on High Speed UART1 which corresponds to pins 6 and 8 of the JP1 header
* -> There is a number one labeled next to the header for reference
* Voltage level is 3.3 for all of the GPIO pins. This is pin 4 on JP1 for 3.3. GND is pin 3.
* Documenation says that UART1 appears as /devttyS4
* Need to downoad minicom to test connection
* -> sudo apt-get install minicom
* -> dmesg
* -> Look for FT230x Basic UART

Problem I had when I got into grub -> grub commands
* normal_exit
* set boot=(hd0,gpt2)
* set prefix=(hd0,gpt2)/boot/grub
* insmod normal
* normal
* -> use ls to find the drives
* -> -> then use ls (hd0, ---)/boot to find which is thre currect one
* -> -> may not be gpt2
* Then in Ubuntu
* -> sudo grub-install /dev/sda
