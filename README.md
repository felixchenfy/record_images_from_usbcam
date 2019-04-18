
Record video images either using Python or ROS.


After running the program,  
press 's' to start recording    
press 'd' to stop recording    

Images will be saved into "data/none_04-18-17-37-33-376/", where the folder name is based on current time.

# 1. Python Version

$ python src/record_video.py

# 2. ROS Version

open usb camera:  
$ roslaunch record_images_from_usbcam start_usb_camera.launch   

start record:  
$ roslaunch record_images_from_usbcam start_recorder.launch action_type:=walk  
(The folder name will be "data/walk/", as specified by "action_type")
