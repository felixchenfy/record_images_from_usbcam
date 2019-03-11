
# Python Version


# ROS Version

open usb camera:  
$ roslaunch record_images_from_usbcam start_usb_camera.launch   

start record:  
$ roslaunch record_images_from_usbcam start_recorder.launch action_type:=walk  
$ roslaunch record_images_from_usbcam start_recorder.launch source:=folder action_type:=kick  

press 's' to start recording  
press 'd' to stop recording  

# ----------------
# Action types for my project
# ----------------

stand
sit
walk
run
squat
jump
wave
kick
