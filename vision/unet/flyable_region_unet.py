from tf_unet import unet, util, image_util
import os

FUNC_DIR = os.path.dirname(os.path.realpath(__file__))
N_CHANNELS = 3
N_CLASS = 2
N_LAYERS = 5
TRAIN_IMG_DIR = os.path.join(FUNC_DIR, 'train\\*.JPG')
#TEST_IMG_DIR = str(Path(FUNC_DIR).resolve().parents[1]) + '\\test'
ckpt_path =  os.path.join(FUNC_DIR, 'checkpoints')

# Total convolutional layers = 2*N_LAYERS - 1 for N_LAYERS down, N_LAYERS-1 up

#preparing data loading
data_provider = image_util.ImageDataProvider(TRAIN_IMG_DIR,
                                             data_suffix='.JPG', mask_suffix='_mask.JPG', shuffle_data=True)
"""
test_provider = image_util.SimpleDataProvider(TEST_IMG_DIR,
                                             data_suffix='.JPG', mask_suffix='_mask.JPG',shuffle_data=True,
"""


#setup & training
net = unet.Unet(layers=N_LAYERS, features_root=64, channels=N_CHANNELS, n_class=N_CLASS)
trainer = unet.Trainer(net=net, batch_size=1, verification_batch_size=1, optimizer='adam')
path = trainer.train(data_provider, ckpt_path, training_iters=10, epochs=100)

"""
#verification - THIS PART HAS NOT BEEN COMPLETED YET.
...
img_data = data_provider[0]
label_data = data_provider[1]
n_img = img_data.shape[0]

prediction = net.predict(path, data_provider[0])

unet.error_rate(prediction, util.label, prediction)

img = util.combine_img_prediction(data_provider[0], data_provider[1], prediction)
util.save_image(img, "prediction.jpg")
"""