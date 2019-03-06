#!/usr/bin/env bash
python object_detection/model_main.py --model_dir=../results/mask-rcnn-inception-v2 --pipeline_config_path=configs/mask-rcnn-inception-v2/train.config --num_train_steps=60000 --alsologtostderr