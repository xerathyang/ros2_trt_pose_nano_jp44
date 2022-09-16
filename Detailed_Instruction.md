# DETAILED INSTRUCTION #
* This guide will give step by step instruction to run the node and get output*

## Start with brand new Jetson Nano ##
After you complete the basic setting on the Jetson Nano, the following setting are used to make sure the process won't break down

- Install require package, include text editor "nano" and swap "dphys-swapfile"

```
$ sudo apt-get update
$ sudo apt-get install nano
$ sudo apt-get install dphys-swapfile
```

- Open the file, find the *CONF_MAXSWAP*, set its value to 4096

```
$ sudo nano /sbin/dphys-swapfile
```

- Open the file, find the *CONF_SWAPSIZE*, set its value to 4096

```
$ sudo nano /etc/dphys-swapfile
```

- Restart the swap service to make the setup become effective

```
$ sudo service dphys-swapfile restart
```

- Now you can check swap size by below command

```
$ free -h
```

- Setting the build runtime

```
$ sudo nano /etc/docker/daemon.json
```

The file should be modified to look like this:

```
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },

    "default-runtime": "nvidia"
}
```


- Then clone this repository

```
$ git clone https://github.com/xerathyang/ros2_trt_pose_nano_jp44.git
```

- Build the docker image

```
$ sudo sh docker_build.sh
```

- Or you got image file, then you can use docker load to import image

```
$ sudo docker load < "file name"
```

## After you finish with the image ##

### Open a terminal to run below command ###

- Open the container by script

```
$ cd /ros2_trt_pose_nano_jp
$ sudo sh docker_run.sh
```

- Inside the container, install dependencies and packages

```
$ cd /ros2_ws
$ rosdep install --from-paths src --ignore-src --rosdistro eloquent -y
$ colcon build
$ source install/local_setup.sh
```

- If you are using ssh connect, run below command

```
$ apt install --reinstall libgl1-mesa-dri
$ ln -sf /usr/lib/aarch64-linux-gnu/libdrm.so.2.4.0 /usr/lib/aarch64-linux-gnu/libdrm.so.2
```

- Run the TRT POSE node

```
$ ros2 run ros2_trt_pose pose-estimation --ros-args -p base_dir:='/ros2_ws/ros2_data/trt_pose/human_pose'
```
*This will take quite a long time, do NOT interrupt the process or it may corrupt the file*

If it appear "waiting for images...", continue do below.

### Open second terminal to run below command ###

- Use this command to get container name
```
$ sudo docker container ps
```

- Use the name acquired from above command to connect container

```
$ sudo docker exec -it <container_name> bash
```

- Run the cam2image node

```
$ ros2 run image_tools cam2image
```

### Open third terminal to run below command ###

- Use the name acquired from above command to connect container

```
$ sudo docker exec -it <container_name> bash
```

- If you are using ssh connect, run below command

```
$ export DISPLAY='<Your host IP>:0.0'
```

- Use below command to setup and run rqt_topic node

```
$ cd /ros2_ws
$ source install/local_setup.sh
$ ros2 run rqt_topic rqt_topic
```

### Open fourth terminal to run below command ###

- Use the name acquired from above command to connect container

```
$ sudo docker exec -it <container_name> bash
```

- If you are using ssh connect, run below command

```
$ export DISPLAY='<Your host IP>:0.0'
```

- Run rviz2 node

```
$ rviz2
```

- Change the config to see the information provide by ros2_trt_pose node
Go to "File->Change config" on Left up side in the window, select "/ros2_ws/src/ros2_trt_pose/ros2_trt_pose/launch/pose-estimation.rviz" file and discard current config.
