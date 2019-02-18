INPUT_TYPE=image_tensor
PIPELINE_CONFIG_PATH='src/configs/ssd-mobilenet-v1-fpn/train.config'
TRAINED_CKPT_PREFIX='results/ssd-mobilenet-v1-fpn/model.ckpt-15000'
EXPORT_DIR='results/ssd-mobilenet-v1-fpn/saved_model_image_tensor'
python src/object_detection/export_inference_graph.py \
    --input_type=${INPUT_TYPE} \
    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
    --trained_checkpoint_prefix=${TRAINED_CKPT_PREFIX} \
    --output_directory=${EXPORT_DIR}
