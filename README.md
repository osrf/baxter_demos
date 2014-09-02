baxter_demos
============

Applications

rosrun baxter_demos ee_position_keyboard.py
Teleoperate the robot’s end effectors through Cartesian space. Press ‘?’ in the terminal you launched it from to view the possible commands.

roslaunch baxter_demos ee_position_joystick joystick:=logitech
Teleoperate the robot’s end effectors through Cartesian space using a joystick/gamepad. Visit http://wiki.ros.org/joy for instructions on configuring your device. 

roslaunch point_input_trajectory.launch limb:=[left/right] img_folder:=[path to asset folder]
This demo will give you instructions on Baxter’s head screen. Move Baxter’s arm to a start configuration using the wrist sensor, then press the round cuff button. Then move Baxter’s arm to an end configuration and press the cuff button again. Baxter will repeatedly move between those two positions. Requires the Joint Trajectory Action Server (see below).

roslaunch baxter_demos object_finder.py topic:=[topic name] limb:=[left/right] method:=[color/star/watershed]
Run a standalone 2D image segmentation routine. The default behavior uses the image feed from Baxter’s right hand camera. It requires you to click on a point in the scene and then uses color segmentation to identify objects of that same color. Three windows will pop up once segmentation begins: the raw image feed, the thresholded image resulting from segmentation processing, and the contours in the thresholded image. The thresholded image window also has sliders that adjust various segmentation parameters. The raw image feed displays the centroid, bounding box, and dominant axis in green for each contour. The other working option for segmentation method is Star Classifier Detection, which uses corner detection to find key points in the image, gets the color of those points and then color segments the image. Make sure the camera topic displays the scene you want to segment before starting object_finder with this segmentation method. object_finder publishes the centroid and dominant axis of each detected object to /object_tracker/blob_info.

roslaunch baxter_demos object_tracker.py limb:=[left/right] method:=[color/star/watershed] folder:=[path to asset folder]
A simple object pick and place demo. Make sure Baxter’s hand camera can see the desired object and click on the object in the image window. Then use the cuff buttons to give Baxter a start and end configuration (make sure the hand camera can see the object in the start configuration). The robot will then use object_finder.py as well as its rangefinder and a visual servoing node to position the gripper over the object, grasp it, move to the end configuration, and drop it.

roslaunch baxter_demos track_ar_tag.launch
Run a node from ar_track _alvar to give the pose of an AR tag in the camera scene. Edit this script with the correct image topic name and dimensions of the marker you are trying to track. See http://wiki.ros.org/ar_track_alvar for more information. To run this node in hydro or higher, you will need to install ar_track_alvar from source. I found that the default size suggested in ar_track_alvar gives very noisy results for an Asus Xtion. However, pose tracking was robust for a marker with a side of nearly 12 cm.

roslaunch baxter_demos get_ar_calib.py
Runs track_ar_tag and an additional node to calculate the transformation between Baxter’s base frame and the camera. To get an accurate extrinsic calibration, you need to carefully measure the transformation between the AR marker and one of Baxter’s coordinate frames and edit $(baxter_demos)/config/ar_calib.yaml with appropriate values.

roslaunch baxter_demos baxter_osrf.launch
Publishes the Baxter URDF with the camera box modification and the saved static transform between the base and camera frames that results from get_ar_calib.py. This is a startup script meant for use with OSRF’s Baxter, so I don’t suggest using it for your own robot unless you modify it first.

roslaunch baxter_demos goal_object_detector.launch kinect:=[true/false]
Runs a standalone 3D image segmentation routine. This node manager will start Openni/Freenect drivers for your 3D camera and show you a registered color point cloud. Shift-click on the object you want to track (make sure that you don’t accidentally click on a background pixel). Enjoy the ensuing mountain of vaguely helpful terminal printouts.
This file relies on baxter_nodelets.xml in the top level of the baxter_demos repo.



Unfinished/questionable functionality
stackit.sh
An unfinished pick-and place demo that combines Moveit, 3D image segmentation, and visual servoing. Run and be amazed at this program’s inability to deal with collisions!
The stackit.sh script runs the Joint Trajectory Action Servo and the baxter_moveit_config GUI before starting the stackit script. If you want to run stackit on its own, use:
roslaunch baxter_demos stackit.launch
(This launchfile includes goal_object_detector.launch and runs stackit.py as its own node.)

super_stacker.launch
An unfinished/extremely broken attempt at 2D pick-and-place stacking.

Tests
object_finder_test.launch
object_visualize_test.launch

Config files
ar_calib.yaml
base_camera_tf.yaml
baxter.urdf
baxter.srdf
object_finder_3d.yaml
object_finder.yaml
servo_to_object.yaml
