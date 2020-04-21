# Compiling
Build like a normal ros2 project
```
cd ~/ros2_ws
colcon build --packages-up-to ros2_gstreamer
```

Copy plugins to
/usr/lib/x86_64-linux-gnu/gstreamer-1.0/

# To run
Use like any normal gstreamer plugin

```
gst-launch-1.0 -v v4l2src device=/dev/video6 ! video/x-raw,framerate=30/1 ! videoconvert ! rclcpp_publisher
```

```
gst-launch-1.0 -v rclcpp_subscriber ! video/x-raw,width=800,height=448,framerate=15/1,format=BGRA ! videoconvert ! xvimagesink
```
