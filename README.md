## mujoco工具箱

### 需求列举

1. 轨迹可视化需求
- 3D模型运动轨迹渲染
- 肌肉激活状态热力图显示(TODO)
- 多视角相机支持(重载render类)
- 支持1080p/4K分辨率(自定义长宽)
- 可调节播放速度和帧率

2. 数据记录需求
- 记录关节位置(qpos)和速度(qvel)
- 记录身体部位位置(xpos)和方向(xquat) 
- 记录肌腱路径点位置(TODO)
- 记录传感器数据(TODO)
- 支持多种数据格式(.npy/.txt/.csv)

3. 轨迹处理需求
- 读取多种格式轨迹数据(.npy/.txt/.mot)(TODO)

4. 命令行工具
- 命令行工具参数配置(python parser)

### 格式规定

建议输出默认为npy格式（多个数组用npz），数组中用(time,data)这样的形式。

### 文件层级规定

- mujoco_tools
  - mujoco_tools
  - models # test model of mujoco humanoid
  - examples

### 用法规定

1. 直接通过命令行操作 

```mujoco-tools -m <model.xml> -d <data_file> -mode <kinematics/dynamics> [options]
Options:
-m, --model        MuJoCo XML model file path
--mode             Simulation mode (kinematics: runs mj.fwd_position, dynamics: runs mj.step)
Input data:
-d, --data          Input data type and path (e.g. qpos /path/to/qpos.npy ctrl /path/to/ctrl.npy)
                    If no data provided, default value of 0 will be used
Visualization options:
-o, --output       Output video path
--resolution       Video resolution (e.g. 1080p, 4K)
--width            Video width in pixels (default: 1920)
--height           Video height in pixels (default: 1080) 
--fps              Video framerate (default: 50)
--timesteps        Timesteps between frames
--camera           Camera names
--flags            Custom vision flags string (e.g. "mjVIS_ACTUATOR mjVIS_ACTIVATION")
Recording options:
--record          Enable data recording
--format          Output format (npy/txt/csv) (default: npy)
--datatype        Data types to record (e.g. "qpos qvel xpos xquat sensor tendon")
                qpos: joint positions
                qvel: joint velocities
                xpos: body positions 
                xquat: body orientations
                sensor: sensor data
                tendon: tendon path points
```
2. 通过bash操作
```
可以通过bash脚本来配置和运行命令，例如:
#!/bin/bash

# 设置默认值
MODEL_PATH="models/humanoid/humanoid.xml"  # MuJoCo模型文件路径
DATA_PATH="qpos data/qpos.npy ctrl /path/to/ctrl.npy"                  # 输入数据文件路径
MODE="kinematics"                          # 仿真模式
OUTPUT="output/video.mp4"                  # 输出视频路径
RESOLUTION="1080p"                         # 视频分辨率
FPS=50                                     # 视频帧率
CAMERA="side"                              # 相机视角
RECORD_DATA=1                              # 是否记录数据
DATA_FORMAT="npy"                          # 输出数据格式
RECORD_TYPES="qpos qvel xpos"              # 要记录的数据类型

# 构建命令
CMD="mujoco-tools \
    -m \"$MODEL_PATH\" \
    -d \"$DATA_PATH\" \
    --mode \"$MODE\" \
    -o \"$OUTPUT\" \
    --resolution \"$RESOLUTION\" \
    --fps \"$FPS\" \
    --camera \"$CAMERA\""

# 添加数据记录相关参数
if [ "$RECORD_DATA" -eq 1 ]; then
    CMD+=" --record"
    CMD+=" --format \"$DATA_FORMAT\""
    CMD+=" --datatype \"$RECORD_TYPES\""
fi

# 执行命令
eval "$CMD"
```
3. 通过python脚本
```
通过load.sh 进入pbd程序，然后就可以用cursor的AI生成代码来生成需要的东西，比如说用来生成ctrl或者qpos的序列，也可以查询其他的东西
```