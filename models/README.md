# YOLO11n 权重、来源与重导出

当前部署仍使用原 `yolo11n.onnx`，本次来源核验没有替换或重新训练模型。
`yolo11n.pt` 与从 [Ultralytics 官方 v8.3.0 资产发行版](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt)
重新获取的文件逐字节一致；两个部署文件的指纹、元数据和验证结果见
[模型清单](model-assets.provenance.json)。

官方预训练权重及本地 ONNX 内嵌元数据均明确标注 AGPL-3.0，完整文本随本目录
[LICENSE-AGPL-3.0](LICENSE-AGPL-3.0) 交付；运行库的 MIT 许可证不替代模型许可。
该来源记录没有声明取得其他商业许可。训练信息来自官方权重及
[YOLO11 说明](https://docs.ultralytics.com/models/yolo11/)：COCO 检测、80 类；
不声明保留原训练集、训练环境、训练日志或能够重现训练本身。

ONNX：FP32，opset 12，静态 `images[1,3,640,640]` 输入、`output0[1,84,8400]` 输出，
80 个类别的顺序与检测配置一致；不包含最终 NMS。
嵌入导出元数据为 Ultralytics 8.3.9、PyTorch 2.8.0、日期 2026-09-12。
原操作者的命令日志未保留。以下是本次实际验证过的等价重导出方法，不能冒称历史命令：

```bash
# 在源码工作区根目录操作，使用独立 Python 3.10 环境。
# 先安装 PyTorch 官方 CPU wheel。
python -m pip install torch==2.8.0+cpu torchvision==0.23.0+cpu \
  --index-url https://download.pytorch.org/whl/cpu
python -m pip install ultralytics==8.3.9 onnx==1.17.0 onnxslim==0.1.71 \
  numpy==1.26.4 opencv-python==4.11.0.86
# 必须复制到新的空目录，避免覆盖部署 ONNX。
mkdir export-review
cp models/yolo11n.pt export-review/
TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1 python - <<'PY'
import hashlib
from pathlib import Path
from ultralytics import YOLO
p = Path('export-review/yolo11n.pt')
assert hashlib.sha256(p.read_bytes()).hexdigest() == \
    '0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1'
YOLO(str(p)).export(format='onnx', opset=12, imgsz=640, batch=1,
                    dynamic=False, half=False, simplify=True, device='cpu')
PY
python tools/source/verify_yolo_export.py export-review/yolo11n.onnx
```

显式允许旧 checkpoint 加载前必须通过上述固定官方 SHA 检查。
本次重导出全部 191 个 initializer 字节一致；忽略诊断名称、内部张量的等价重命名和
可选的中间形状注释后，算子、属性、连接、权重、输入／输出类型和形状完全一致。
部署文件 IR 为 10，本次重导出 IR 为 7；两者都通过 ONNX 结构检查且处于运行库支持范围。
文件 SHA 不相同，包含 IR、日期及注释差异；核验工具不会忽略数值或计算差异。

检测包安装模型、许可、此说明及模型清单。PT 仅用于审查／重导出，不是运行时依赖。
来源核验不改变既有识别质量结论；卡车近距离车尾漏检仍由未完成清单 V-09 保留。
