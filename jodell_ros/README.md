# JODELL ROS 驱动

## 启动
 ``` bash
roslaunch jodell_ros start.launch
 ```

## 反馈话题
### /claw_status
``` bash
int32 status #状态，暂时没用

int32 position # 角度，数值 0-255

int32 speed # 速度，数值 0-255
int32 torque # 夹爪力矩，数值 0-255
int32 temp # 温度
int32 voltage # 电压
```

## 控制话题
### /set_angle
``` bash
# 位置： 0-255
int32 position

# 速度：0-255
int32 speed

# 力矩：0-255
int32 torque
```