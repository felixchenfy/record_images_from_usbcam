
Record video images either using Python or ROS.


After running the program,  
press 's' to start recording    
press 'd' to stop recording    


# 1. Python Version

$ python src/record_video.py action_type:=walk  

# 2. ROS Version

open usb camera:  
$ roslaunch record_images_from_usbcam start_usb_camera.launch   

start record:  
$ roslaunch record_images_from_usbcam start_recorder.launch action_type:=walk  

