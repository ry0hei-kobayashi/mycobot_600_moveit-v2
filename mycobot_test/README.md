# mycobot_pro_600をMoveIt2で使用するためのコンフィグ　
　（under construction）

## 環境セットアップ
### ros_humble環境
```
cd <ros>_ws/src
git clone https://github.com/ry0hei-kobayashi/mycobot_600_moveit.git
cd .. && colcon build
```

### 非ros_humble環境
```
cd <任意のdirectory>
git clone https://github.com/ry0hei-kobayashi/mycobot_600_moveit.git
cd docker && docker compose build
xhost local:
docker compose up
```
```
#terminal 2
docker exec -it <container_name> bash   #log in to container
tmux
source install/setup.bash
```

## 使用方法

```
#xacro to urdf
ros2 run xacro xacro hma_cobot_with_eef.xacro -o hma_cobot_with_eef.urdf

#check your urdf
check_urdf hma_cobot_with_eef.urdf

colcon build
source install/setup.bash

#moveitのセットアップファイルを起こす
ros2 launch mycobot_600_moveit setup_assistant.launch.py

#コンフィグのテストを行う
ros2 launch mycobot_600_moveit demo.launch.py
```

## 使用方法
編集を行う場合は，config/moveit\_controller.yamlとconfig/ros2\_controller.yaml, config/mycobot_pro_with_gripper.srdfを削除すること

## 作成したコンフィグを実機で試す
```
roslaunch mycobot_600_moveit demo.launch
rosrun mycobot_600_moveit sync_plan.py
```
