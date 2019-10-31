# Record video images either using Python or ROS.

After running the program,    
press `s` to start recording;  
press `d` to stop recording.  


# 1. Python Version
```
python src/record_video.py \
    --frame_rate 10.0 \
    --webcam_index 0 \
    --sub_folder_name none
```

Images will be saved to `"data/${sub_folder_name}_${current_time}"`, e.g.: `"data/none_04-18-17-37-33-376/"`.  

# 2. ROS Version

```
roslaunch record_images_from_usbcam record_images.launch \
    frame_rate:=10.0 \
    sub_folder_name:=none  
```
