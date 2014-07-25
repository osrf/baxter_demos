"""Module for stereovision on the Baxter robot by Jackie Kay """

"""Arguments: alias for cameras we are using: <left_camera, right_camera, head_camera, left, right, head, left_hand, right_hand>

/robot/cameras/<name>_camera
/robot/limb/<hand>/endpoint_state

Can we assume that the head camera stays in a fixed position?

setup:
Get camera locations relative to end effector topics from URDF

main loop:
calibrate(): Based on known poses from endpoint_state and URDF (with help from TF?), figure out the rotation and translation between the two camera frames
R1, R2, P1, P2, Q = cv2.stereoRectify(cam_matrix1, cam_matrix2, distCoeffs1, distCoeffs2, imageSize, R, T)
points4d = cv2.triangulatePoints(P1, P2, projPoints1, projPoints2) #homogeneous coordinates
"""
