# 外部工具接入资料

[qgc/](qgc/README.md) 保存 QGroundControl 的本地文件身份和两种 EasyFly 接入方式说明。
该目录不包含 QGC 源码、安装器或用户二进制，不参与 ROS 包发现。

程序由用户在连接配置、文件选择器或已有命令行参数中显式指定。
本机 `src/third_party/QGC.AppImage` 保持原位置并继续忽略；文档分类不改变运行程序路径。
启动、隔离、联网维护、会话所有权与正常关闭仍由 EasyFly UI 层实现。

迁移记录见 [ALG-8.4](../../algorithms/alg8-4-validation.md)。
