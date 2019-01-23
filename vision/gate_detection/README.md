## Gate Detection
Use tensorflow object detection API to build a simple gate detector for demonstration purpose. Let's use COCO format as our primary format. The door data is extracted from Google Open Image.

## How to run the code

1. Download the door data from dropbox and put the tfrecord files into folder "images"
2. Download [tensorflow/models](https://github.com/tensorflow/models) into third_party folder
3. Run set_up_env.sh. This will set up path for tensorflow/models
4. Run ```python -m src.make_train_config```. This will write path into template.config and generate train.config.
5. Go to "src" folder and run ```sh submit_local_training.sh```
6. Go to "results" folder and use tensorboard to visualize results

Note: 
* Run the code with GPUs. 
* Use anaconda to manage environment.
* Use Python 3.6
