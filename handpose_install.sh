sudo -H pip3 install --upgrade pip
pip3 install https://files.pythonhosted.org/packages/15/cd/c2061d9050d4f19da2849e54161fb5b420d39236b08f4d51cc3e90de2d7f/scikit_learn-0.24.2-cp36-cp36m-manylinux2014_aarch64.whl
pip3 install numpy==1.19.4
cd /
git clone https://github.com/NVIDIA-AI-IOT/trt_pose_hand.git
cd /ros2_ws/src
git clone https://github.com/NVIDIA-AI-IOT/ros2_trt_pose_hand.git
rm -rf ros2_trt_pose
cd /ros2_ws/ros2_data
mkdir -p trt_pose_hand/hand_pose
cd trt_pose_hand/hand_pose
gdown 1NCVo0FiooWccDzY7hCc5MAKaoUpts3mo
cp /trt_pose_hand/svmmodel.sav svmmodel.sav
cp /trt_pose_hand/preprocess/gesture.json gesture.json
cp /trt_pose_hand/preprocess/hand_pose.json hand_pose.json
cd /ros2_ws