# aruco_ros_opencv
This is a ROS package for pose detection using aruco markers.
Copy this package to the src directory of catkin workspace and make the project
Run usb_cam.launch file to run usb_cam node and aruco_detection node.
This package have used opencv techniques for pose estimation of aruco markers.


aruco_detection node:

Subscriptions:
/usb_cam/camera_info (sensor_msgs/CameraInfo)
/usb_cam/image_rect_color (sensor_msgs/Image)

Publications:
/aruco/marked_image_out (sensor_msgs/Image)
