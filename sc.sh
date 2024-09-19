#!/bin/bash

# 获取当前主机名
HOSTNAME=$(hostname)

# 获取可选的 commit 消息
COMMIT_MESSAGE=${1}
if [ -z "$COMMIT_MESSAGE" ]; then
    # 如果没有提供 commit 消息，使用默认消息
    COMMIT_MESSAGE="wip: $(date +'%Y-%m-%d %H:%M:%S')"
fi

# 打印当前操作的信息
echo "Commit message: $COMMIT_MESSAGE"
echo "Current host: $HOSTNAME"

if [[ "$HOSTNAME" == *"Jing"* ]]; then
    # 如果主机名包含 'WSL'，则在 WSL 中运行的操作
    echo "Running in WSL. Performing Git operations..."
    
    # 添加所有文件到暂存区
    git add .
    
    # 提交更改
    git commit -m "$COMMIT_MESSAGE"
    
    # 推送到远程仓库
    git push origin
elif [[ "$HOSTNAME" == *"nano"* ]]; then
    # 如果主机名包含 'linux'，则在 Linux 主板中运行的操作
    echo "Running in Linux. Performing Git operations..."
    
    # 强制重置当前分支到最新的 HEAD
    git reset --hard HEAD
    
    # 拉取远程最新代码
    git pull origin
    
    # 运行 catkin_make
    catkin_make
else
    echo "Unknown host. Please run this script on WSL or Linux."
    exit 1
fi