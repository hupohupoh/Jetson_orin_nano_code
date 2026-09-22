#!/bin/bash
# GPU服务器上训练YOLOv8n几何图卡检测
# 用法: bash train_yolo.sh [epochs]
EPOCHS=${1:-100}

set -e
cd /root

# 解压数据集
if [ ! -d /root/synthetic_dataset ]; then
    tar xzf synthetic_dataset.tar.gz
fi

# 修正 dataset.yaml 路径（指向服务器实际路径）
cat > /root/synthetic_dataset/dataset.yaml << 'EOF'
path: /root/synthetic_dataset
train: images
val: images
names:
  0: circle
  1: pentagon
  2: square
  3: diamond
  4: cross
  5: triangle
EOF

# 合并所有批次（YOLO train 直接指 images 目录，把批次合并）
cd /root/synthetic_dataset
if [ ! -d images_all ]; then
    mkdir -p images_all labels_all
    for b in batch_*/; do
        cp -n "$b/images/"* images_all/ 2>/dev/null || true
        cp -n "$b/labels/"* labels_all/ 2>/dev/null || true
    done
    echo "合并完成: $(ls images_all | wc -l) 张图"
fi

# 用合并后的目录训练
cat > /root/synthetic_dataset/dataset_all.yaml << 'EOF'
path: /root/synthetic_dataset
train: images_all
val: images_all
names:
  0: circle
  1: pentagon
  2: square
  3: diamond
  4: cross
  5: triangle
EOF

echo "=== 开始训练 YOLOv8n, ${EPOCHS} epochs ==="
nohup yolo detect train data=/root/synthetic_dataset/dataset_all.yaml \
    model=yolov8n.pt \
    epochs=${EPOCHS} imgsz=640 batch=32 \
    project=/root/yolo_runs name=shape_train \
    > /root/train.log 2>&1 &

echo "训练已后台启动 (PID $!)"
echo "日志: tail -f /root/train.log"
echo "查看进度: watch -n 5 'tail -20 /root/train.log'"
