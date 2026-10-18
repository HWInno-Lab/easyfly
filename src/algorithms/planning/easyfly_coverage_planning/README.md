# easyfly_coverage_planning

项目维护的 C++17 矩形蛇形覆盖路线和已完成条带面积计算，迁自任务状态机包。
仅依赖标准库，导出 `easyfly_coverage_planning::easyfly_coverage_planning`。
公开头为 `easyfly_coverage_planning/search_geometry.hpp`，命名空间 `easyfly::coverage`。

保留原 SearchPoint/SearchBounds/searchRoute、尺寸/间距/高度/边界校验和 256 条带上限。
每条带返回进入点和结束点；进入/转场不计覆盖面积，结束点才增加该条带面积。
几何计算按调用者传入的边界校验；任务节点继续负责实时飞行准入、Action 所有权、
段执行、取消和进度发布。没有新增覆盖规划 API 包或隐式避障功能。
