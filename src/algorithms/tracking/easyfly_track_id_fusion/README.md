# easyfly_track_id_fusion

世界坐标辅助 ID 整合，使用显式 steady 秒、同类别和半径/TTL 墓园。
头文件 `easyfly_track_id_fusion/id_consolidator.hpp`，命名空间
`easyfly::track_id_fusion`，target 为 `easyfly_track_id_fusion::easyfly_track_id_fusion`。
只有标准库依赖；调用方串行调用、接纳坐标/时间/参考系并管理 reset。

原 10 条回归用例迁入本包。ALG-2 另补延迟坐标回流与 reset 用例：新 raw ID 在首次
出现尚无世界观测时保持临时自身映射；首次真实世界坐标回流时允许一次旧墓园匹配。
已有真实观测或已继承旧 ID 的活跃轨迹不再被重新分配。类别、距离和 TTL 规则保持。

这是原完整 ROS 回流链的功能修复，不是纯目录移动；迁移前旧实现会在该延迟场景保持新 ID。
不表示外观 ReID、复杂遮挡身份正确性或实机资格。
