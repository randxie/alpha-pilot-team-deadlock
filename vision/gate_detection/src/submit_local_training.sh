#!/usr/bin/env bash
#python object_detection/model_main.py --model_dir=../results/ssd-mobilenet-v1-fpn --pipeline_config_path=configs/ssd-mobilenet-v1-fpn/train.config --num_train_steps=80000 --sample_1_of_n_eval_examples=1 --alsologtostderr

python object_detection/model_main.py --model_dir=../results/ssd-resnet50-v1-fpn --pipeline_config_path=configs/ssd-resnet50-v1-fpn/train.config --num_train_steps=80000 --sample_1_of_n_eval_examples=10 --alsologtostderr


# get tensorboard
# tensorboard --logdir=../results/ssd-mobilenet-v1-fpn