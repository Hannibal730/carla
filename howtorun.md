# ros2_sensor.py 실행 순서 (lidar_2d 포함, 선택적 센서 활성화)

## 1번 터미널: CARLA 서버 (동일)

cd ~/carla
source /opt/ros/humble/setup.bash

__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ./CarlaUE4.sh -RenderOffScreen -quality-level=Low --ros2


## 2번 터미널: 수동 조종 차량 (동일)

cd ~/carla
source .venv/bin/activate

python PythonAPI/util/config.py --no-rendering

python PythonAPI/util/config.py --map Town01_Opt

python PythonAPI/examples/manual_control.py --rolename car --filter vehicle.micro.microlino --generation 2 --sync


## 3번 터미널: ROS2 센서 publisher
# 모든 센서 활성화
cd ~/carla
source /opt/ros/humble/setup.bash
source .venv/bin/activate

python PythonAPI/examples/ros2_sensor/ros2_sensor.py \
  -f PythonAPI/examples/ros2_sensor/stack.json \
  --attach-existing \
  --passive \
  --python-ros2 \
  --base-frame base_link \
  --wait-for-vehicle 30




# 원하는 센서만 선택 (예시)

cd ~/carla
source /opt/ros/humble/setup.bash
source .venv/bin/activate

python PythonAPI/examples/ros2_sensor/ros2_sensor.py \
  -f PythonAPI/examples/ros2_sensor/stack.json \
  --attach-existing \
  --passive \
  --python-ros2 \
  --base-frame base_link \
  --wait-for-vehicle 30 \
  --sensors lidar_2d gnss




# 주의: --python-ros2 를 반드시 포함해야 static TF(base_link → 각 센서)가
#       퍼블리시된다. 없으면 RViz에서 아무것도 렌더링되지 않는다.
#       이 노드는 CARLA simulation clock(/clock)도 발행한다.

## 4번 터미널: RViz



source /opt/ros/humble/setup.bash

rviz2 -d /home/hannibal/carla/PythonAPI/examples/ros2_sensor/rviz/ros2_sensor.rviz --ros-args -p use_sim_time:=true






## 종료
pkill -TERM -f 'ros2_sensor.py'
pkill -TERM -f 'manual_control.py'
pkill -TERM -f 'rviz2'
pkill -TERM -f 'CarlaUE4-Linux-Shipping'


## microlino 바퀴 위치 정보.
wheel[0]: x=17.492 y=69.149 z=0.250
wheel[1]: x=17.490 y=70.340 z=0.250
wheel[2]: x=15.967 y=69.380 z=0.250
wheel[3]: x=15.965 y=70.102 z=0.250
