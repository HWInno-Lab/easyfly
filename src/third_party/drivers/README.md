# 固定 RealSense 驱动的离线恢复

顶层 gitlink 指向 `d69f05c1b4a3f0d5740e25d6429d4417c7f3d75d`，其中包含一次
EasyFly 构建适配；该提交没有被声明为官方远端上的提交。
其父提交 `bafc21080c5c8e259dadbb309797949aee0dd950` 已从
[官方仓库](https://github.com/IntelRealSense/realsense-ros/commit/bafc21080c5c8e259dadbb309797949aee0dd950)
独立获取并核对。原 Apache-2.0 许可和各文件通知全部保留。

新检出先执行以下步骤，随后再运行 colcon：

```bash
git clone --no-recurse-submodules <EasyFly 仓库地址> easyfly_ws
cd easyfly_ws
python3 tools/source/restore_realsense.py
```

恢复工具使用随源码提供的完整 Git bundle，检查 SHA256、提交、源码树和 Git 对象；
Git 外部网络协议被禁用，不依赖本机旧仓库或已缓存对象。
`.gitmodules` 记录可信官方来源，但普通递归克隆不能从官方远端取得这个本地适配提交，
因此上述恢复命令是本版本的正式获取流程。

目标目录为空时才创建新检出；正确且干净的已有检出重复执行不变。
已有修改、其他提交、非仓库目录或 bundle 损坏均拒绝，保留原目录。
`--destination PATH` 可以把同一固定驱动恢复到另一个空目录，用于审计或离线构建。
工具不安装设备规则、内核驱动或系统包，也不声明相机硬件已验收。

版本、bundle 和本地补丁指纹见 [来源清单](realsense-ros.provenance.json)；
[适配差异](realsense-workspace.patch)移除旧 SDK rosdep 依赖，并用 `COLCON_IGNORE`
保留当前不参与工作空间构建的三个组件。恢复后仍可从官方远端手动审查升级，
升级不会由本工具自动触发。
