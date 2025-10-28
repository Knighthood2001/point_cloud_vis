通过这个命令，我们可以查看点云消息的字段信息。
```shell
rostopic echo /points_raw1/fields -n 1
```

```shell
wu@wu:~/catkin_ws$ rostopic echo /points_raw1/fields -n 1
- 
  name: "x"
  offset: 0
  datatype: 7
  count: 1
- 
  name: "y"
  offset: 4
  datatype: 7
  count: 1
- 
  name: "z"
  offset: 8
  datatype: 7
  count: 1
- 
  name: "intensity"
  offset: 12
  datatype: 7
  count: 1
- 
  name: "ring"
  offset: 16
  datatype: 4
  count: 1
- 
  name: "time"
  offset: 18
  datatype: 7
  count: 1
---
```
我这里只获取xyz信息。

```shell
cd ~/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="point_cloud_vis"
```