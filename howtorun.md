# CARLA ROS2 Native 실행 순서

## 1번 터미널: CARLA 서버
cd ~/carla
source /opt/ros/humble/setup.bash

__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
./CarlaUE4.sh -RenderOffScreen -quality-level=Low --ros2


## 2번 터미널: 수동 조종 차량
cd ~/carla
source .venv/bin/activate

python PythonAPI/examples/manual_control.py --rolename car


## 3번 터미널: ROS2 센서 publisher
cd ~/carla
source /opt/ros/humble/setup.bash
source .venv/bin/activate

python PythonAPI/examples/ros2/ros2_native.py \
  -f PythonAPI/examples/ros2/stack.json \
  --attach-existing \
  --passive \
  --python-ros2 \
  --base-frame base_link \
  --wait-for-vehicle 30

## 4번 터미널: RViz

source /opt/ros/humble/setup.bash

rviz2 -d /home/hannibal/carla/PythonAPI/examples/ros2/rviz/ros2_native.rviz



## 종료

각 터미널에서 `Ctrl+C`로 종료하는 것이 가장 안전하다.

한 번에 정리해야 하면:

pkill -TERM -f 'ros2_native.py'
pkill -TERM -f 'manual_control.py'
pkill -TERM -f 'rviz2'
pkill -TERM -f 'CarlaUE4-Linux-Shipping'

