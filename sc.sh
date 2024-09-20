#!/bin/bash

# 获取当前主机名
HOSTNAME=$(hostname)

# 默认的 commit 消息
COMMIT_MESSAGE="wip: $(date +'%Y-%m-%d %H:%M:%S')"
TAG=""

# 解析参数
while getopts "m:t:" opt; do
    case ${opt} in
        m )
            COMMIT_MESSAGE=$OPTARG
            ;;
        t )
            TAG=$OPTARG
            ;;
        \? )
            echo "Usage: cmd [-m commit_message] [-t tag]"
            exit 1
            ;;
    esac
done

# 打印当前操作的信息
echo "Commit message: $COMMIT_MESSAGE"
echo "Tag: $TAG"
echo "Current host: $HOSTNAME"

if [[ "$HOSTNAME" == *"Jing"* ]]; then
    # 如果主机名包含 'WSL'，则在 WSL 中运行的操作
    echo $"Running in WSL. Performing Git operations...\n"

    # 检查是否有更改需要提交
    if [[ -z $(git status --porcelain) ]]; then
        echo "No changes to commit. Exiting."
        exit 0
    fi
    
    # 添加所有文件到暂存区
    git add .
    
    # 提交更改
    git commit -m "$COMMIT_MESSAGE"
    
    # 如果指定了 tag，则添加 tag
    if [ -n "$TAG" ]; then
        git tag "$TAG"
        echo "Tag '$TAG' created."
    fi
    
    # 推送到远程仓库
    git push origin
    # 推送 tag
    if [ -n "$TAG" ]; then
        git push origin "$TAG"
    fi

elif [[ "$HOSTNAME" == *"nano"* ]]; then
    # 如果主机名包含 'linux'，则在 Linux 主板中运行的操作
    echo $"Running in Linux. Performing Git operations...\n"
    
    # 强制重置当前分支到最新的 HEAD
    git reset --hard HEAD
    
    # 拉取远程最新代码
    git pull origin
    
    # 运行 catkin_make
    # catkin_make
else
    echo "Unknown host. Please run this script on WSL or Linux."
    exit 1
fi
