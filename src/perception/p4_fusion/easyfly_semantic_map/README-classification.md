# Independent obstacle class copies

`classification.launch.py use_sim_time:=false` starts `obstacle_classifier`, not
another geometry map. The original `semantic_map_node.cpp` is unchanged.
Inputs `/obstacles` and measured `/target_positions` produce only
`/obstacles_classified` (ClassifiedObstacleArray). Original geometry ID/header,
shape, dynamic flag and velocity are copied; only the independent class metadata
and copy's obstacle_type change. No occupancy or flight output is written.

Expanded AABB association requires unique candidates in both directions, valid
same-frame/time observations, finite upstream confidence above a threshold,
and bounded age/TTL. Unknown or ambiguous classes preserve valid geometry; bad
geometry makes geometry_valid false. Consumers must check geometry_valid,
association_valid and expires_at separately, never discard obstacles by class.
Observed time and expiry are source based, not refreshed by republishing.
Upstream confidence is preserved, not claimed to be a calibrated probability.

Read-only defaults: `max_age=.5`, `association_ttl=.5`, `association_margin=.3`,
`min_confidence=.7`, `time_tolerance=.15`; each input at most 256 items. Clock
zero/pause/rollback, receipt and source freshness are enforced.
`/platform/classification/capabilities` is ready only with a current association.
See [D4 contracts](../../../../docs/development/d4-perception.md) for exact bounds.

Pure association/copy/ambiguity cases, real ROS invalid/lost/expiry cases,
simulated-clock regression and existing map core tests are separate from lint.
No actual sensor/classifier accuracy or flight safety acceptance is claimed.

## ALG-6 算法边界

expanded-AABB 双向唯一类别匹配迁入 easyfly_obstacle_association，仅接收普通几何/类别并返回索引。core.py 继续接纳 header/geometry、复制完整原障碍、组装分类原因及 TTL；不改原障碍、占据图或飞行输入。
