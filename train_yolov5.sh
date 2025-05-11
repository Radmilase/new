#!/bin/bash
python3 /home/ub/yolov5/train.py \
  --img 640 \
  --batch 4 \
  --epochs 1 \
  --data /home/ub/p/data.yaml \
  --weights yolov5s.pt \
  --name custom_train \
  --cache disk \
  --device cpu \
  --workers 2 \
