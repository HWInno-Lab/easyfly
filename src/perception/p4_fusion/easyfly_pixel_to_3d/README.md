# easyfly_pixel_to_3d

## ALG-6 算法边界

中位深度及针孔反投影已迁入 easyfly_projection，普通视图借用深度字节。节点保留编码/端序、图像配对、TF、消息/frame contract 和可视化。原有 latest TF 回退仍保留；不将其描述为雷达相机两源精确时间契约。布局/非有限框新增显式拒绝，正常数学保持原版。
