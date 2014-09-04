rosrun baxter_interface joint_trajectory_action_server.py &
roslaunch baxter_moveit_config demo_baxter.launch &
#Pause for a moment
sleep 5
roslaunch baxter_demos stackit.launch
