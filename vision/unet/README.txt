This is the U-net implementation for flyable region detection.

To train:
1. All the Test 2 training and validation images must be in alpha-pilot\vision\final_code\training\images

2. Run splt_train_val_4unet.py. This will also create the mask jpgs.

3. Run flyable_region_unet.py to train the model. This is still in the works, so the code is barebones.

TF model code is based on:
https://github.com/jakeret/tf_unet

Documentation:
https://tf-unet.readthedocs.io/en/latest/tf_unet.html

There is no need for a setup_py at this time as suggested by the above documentation.
The documentation also has errors in it, so please look into the code to find answers.