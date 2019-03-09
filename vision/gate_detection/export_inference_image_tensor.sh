INPUT_TYPE=image_tensor
PIPELINE_CONFIG_PATH='src/configs/mask-rcnn-inception-v2/train.config'
TRAINED_CKPT_PREFIX='results/mask-rcnn-inception-v2/model.ckpt-52764'
EXPORT_DIR='results/mask-rcnn-inception-v2/saved_model_image_tensor'
python src/object_detection/export_inference_graph.py \
    --input_type=${INPUT_TYPE} \
    --pipeline_config_path=${PIPELINE_CONFIG_PATH} \
    --trained_checkpoint_prefix=${TRAINED_CKPT_PREFIX} \
    --output_directory=${EXPORT_DIR}
