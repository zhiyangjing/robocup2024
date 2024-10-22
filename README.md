## 编译
```bash
# 初次编译
catkin_make -DCATKIN_WHITELIST_PACKAGES=lslidar_msgs

catkin_make -DCATKIN_WHITELIST_PACKAGES=

# 后续编译
catkin_make
```

##  统计代码函数
```bash
find ./src -not -path "./src/cv_bridge/*" -not -path "./src/lsx10/*" -name "*.cpp" -exec cat {} + | wc -l
# 除去外部库之外的cpp文件总行数


```

