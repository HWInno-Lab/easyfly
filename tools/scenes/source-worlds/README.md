# v2 重建输入

此目录只保存医院与赛道 v2 派生过程必需的固定上游字节，不注册为场景，
也不安装到 `easyfly_gazebo_worlds` 的运行目录。

- `hospital_aws_v1.world`：供 `prepare_hospital_world.py` 重现三处高度修订。
  许可与来源见 `src/simulation/easyfly_gazebo_models/licenses/aws_hospital/`。
- `racetrack_aws_day_v1.world`：供 `prepare_racetrack_cold_world.py` 重现保存时钟字段的移除。
  许可与来源见 `src/simulation/easyfly_gazebo_models/licenses/aws_racetrack/`。

两文件由原运行目录原字节迁移，哈希不变。迁移对应关系记录在
`src/simulation/easyfly_gazebo_worlds/evidence/retired-scenes-20261030.json`。
历史导入／几何证据中的路径保留其采集时的布局，不改写旧验证结果。
