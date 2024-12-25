# MuJoCo 工具箱

[English](README.md) | [中文](README_CN.md)

用于 MuJoCo 仿真、可视化和数据处理的综合工具包。

## 功能清单

### 已完成功能
- [x] 基础 MuJoCo 模型加载和仿真
- [x] 功能完备的命令行界面
- [x] 3D 模型运动轨迹渲染
- [x] 多视角相机支持
- [x] 可自定义分辨率（支持高达 4K）
- [x] 可调节播放速度和帧率
- [x] 关节位置（qpos）和速度（qvel）记录
- [x] 身体位置（xpos）和方向（xquat）记录
- [x] 支持多种数据格式（.npy/.txt/.csv）
- [x] 基础数据分析和处理工具

### 开发中功能
- [ ] 添加测试脚本以测试包功能
- [ ] 肌肉激活状态热力图显示
- [ ] 肌腱路径点记录
- [ ] 传感器数据记录
- [ ] 支持 .mot 轨迹格式
- [ ] 高级数据分析工具
- [ ] 文档改进
- [ ] 单元测试
- [ ] 常用场景示例脚本

## 安装

```bash
# 从源代码安装
git clone https://github.com/yourusername/mujoco_tools.git
cd mujoco_tools
pip install -e .
# 从 PyPI 安装
pip install mujoco-tools
# 从 github 安装
pip install git+https://github.com/ShanningZhuang/mujoco_tools.git
```

## 项目结构

```
mujoco_tools/
├── mujoco_tools/         # 主包目录
│   ├── cli.py            # 命令行接口
│   ├── mujoco_loader.py  # MuJoCo 模型加载工具
│   ├── player.py         # 可视化播放器
│   ├── recorder.py       # 数据记录工具
│   ├── tools.py          # 通用工具
│   └── data_processor.py # 数据处理工具
├── models/               # 测试模型
└── examples/             # 示例脚本
```

## 使用方法

### 1. 命令行接口

```bash
mujoco-tools -m <model.xml> [选项]
```

#### 必需参数：
- `-m, --model`：MuJoCo XML 模型文件路径
- `--mode`：仿真模式（kinematics：运行 mj.fwd_position，dynamics：运行 mj.step）[默认：kinematics]

#### 输入数据选项：
- `-d, --data`：输入数据类型和路径（例如："qpos data/qpos.npy ctrl data/ctrl.npy"）
- `--input_data_freq`：输入数据频率 [默认：50]

#### 输出路径选项：
- `--output_path`：输出路径 [默认：logs]
- `--output_prefix`：输出前缀 [默认：output]

#### 可视化选项：
- `--record_video`：启用视频录制
- `--width`：视频宽度（像素）[默认：1920]
- `--height`：视频高度（像素）[默认：1080]
- `--fps`：视频帧率 [默认：50]
- `--output_video_freq`：输出视频频率 [默认：50]
- `--camera`：相机名称 [默认：Free]
- `--flags`：自定义视觉标志（例如："mjVIS_ACTUATOR mjVIS_ACTIVATION"）

#### 记录选项：
- `--record_data`：启用数据记录
- `--format`：输出格式（npy/txt/csv）[默认：npy]
- `--datatype`：要记录的数据类型（空格分隔：qpos qvel xpos xquat sensor tendon）[默认：qpos]
- `--output_data_freq`：输出数据频率 [默认：50]

### 2. Bash 脚本使用

创建配置用的 bash 脚本：

```bash
#!/bin/bash

# 默认设置
MODEL_PATH="models/humanoid/humanoid.xml"
DATA_PATH="qpos data/qpos.npy"
MODE="kinematics"
OUTPUT="output/video.mp4"
RESOLUTION="1080p"
FPS=50
CAMERA="side"
RECORD_DATA=1
DATA_FORMAT="npy"
RECORD_TYPES="qpos qvel xpos"

# 构建命令
CMD="mujoco-tools \\
    -m \"$MODEL_PATH\" \\
    -d \"$DATA_PATH\" \\
    --mode \"$MODE\" \\
    -o \"$OUTPUT\" \\
    --resolution \"$RESOLUTION\" \\
    --fps \"$FPS\" \\
    --camera \"$CAMERA\""

# 添加记录选项
if [ "$RECORD_DATA" -eq 1 ]; then
    CMD+=" --record"
    CMD+=" --format \"$DATA_FORMAT\""
    CMD+=" --datatype \"$RECORD_TYPES\""
fi

# 执行命令
eval "$CMD"
```

### 3. Python 模块使用

```python
# 直接导入模块
from mujoco_tools import MujocoLoader, Player, Recorder

# 命令行方式使用
python -m mujoco_tools.cli -m /path/to/model.xml -d 'qpos /path/to/data.npy'
python -m mujoco_tools.mujoco_loader -m /path/to/model.xml
```

## 数据格式

默认输出格式为 `.npy`（多个数组使用 `.npz`）。数据以 `(time, data)` 格式存储。

## 开发

### 设置开发环境
```bash
# 克隆仓库
git clone https://github.com/yourusername/mujoco_tools.git
cd mujoco_tools

# 创建并激活虚拟环境（可选）
python -m venv venv
source venv/bin/activate  # Linux/Mac
# 或
.\venv\Scripts\activate  # Windows

# 安装开发依赖
pip install -e ".[dev]"
```

### 发布
当在 GitHub 上创建新的发布版本时，包会自动发布到 PyPI。发布新版本的步骤：

1. 在 `setup.py` 中更新版本号
2. 创建并推送新标签：
```bash
git tag v0.1.0  # 使用适当的版本号
git push origin v0.1.0
```
3. 在 GitHub 上使用该标签创建新的发布版本
4. GitHub Action 将自动构建并发布到 PyPI

### 运行测试
```bash
pytest tests/
```

## 贡献

欢迎提交贡献！请随时提交 Pull Request。

贡献步骤：
1. Fork 本仓库
2. 创建您的特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交您的更改 (`git commit -m '添加一些很棒的特性'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启一个 Pull Request

## 许可证

[MIT 许可证](LICENSE) 