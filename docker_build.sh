sudo cp /etc/apt/trusted.gpg.d/jetson-ota-public.asc .
sudo docker build -t ros2_trt_pose_nano:jp44 -f dockerfile.ros.eloquent.trt_pose_nano_jp44 .