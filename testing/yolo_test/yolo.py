# pip install --user -U ultralytics --no-cache-dir
# pip install --user opencv-python-headless onnx==1.15.0
import sys
print(sys.path)

import torch
print(torch.__version__)
print(torch.cuda.is_available())

import ultralytics
ultralytics.checks()
