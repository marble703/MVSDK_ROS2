unset CONDA_DEFAULT_ENV
unset CONDA_PREFIX
unset CONDA_PROMPT_MODIFIER
unset CONDA_PYTHON_EXE
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "miniconda3" | tr '\n' ':' | sed 's/:$//')

# 重新设置 PATH
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# 添加 ROS 2 到 PATH
export PATH="/opt/ros/humble/bin:$PATH"

# 选择 setup 文件
ROS_SETUP=""
LOCAL_SETUP=""

if [ -n "$ZSH_VERSION" ]; then
    ROS_SETUP="/opt/ros/humble/setup.zsh"
    LOCAL_SETUP="install/setup.zsh"
elif [ -n "$BASH_VERSION" ]; then
    ROS_SETUP="/opt/ros/humble/setup.bash"
    LOCAL_SETUP="install/setup.bash"
else
    # 默认使用 bash
    ROS_SETUP="/opt/ros/humble/setup.bash"
    LOCAL_SETUP="install/setup.bash"
fi

#加载 ROS 2 环境
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    echo "❌ ROS 2 setup file not found: $ROS_SETUP"
    return 1
fi

# 加载本地工作空间
if [ -f "$LOCAL_SETUP" ]; then
    source "$LOCAL_SETUP"
fi

echo "env set down"