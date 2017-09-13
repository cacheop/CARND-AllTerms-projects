import numpy as np
import sklearn
import os
import csv
import cv2
import math
import numpy as np
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
from random import random
from random import random

total_cameras = 3 #use right and left cameras too
correction = 0.20 #angle correction for left/right cameras
batch_size = 16 
epoch = 15

# chosen model
#CNN_model = "LeNet"
CNN_model = "NVIDIA"
#CNN_model = "comma_ai"
# training dataset 

#path to data
user_path = '/Users/albertoescarlate/Dropbox/Udacity/_SDC_ND013/CARND-AllTerms-projects/CARND-Term1-projects/P3_data'
base_path = user_path + '/data_udacity'

# resizing values 
IMAGE_W_H = 64 # 64 X 64

from random import random

# flip images horizontally
def flip_image(img, angle):
    img = cv2.flip(img, 1)
    angle = angle * -1.0
    return img, angle

def modify_brightness(img):
    img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    img = np.array(img, dtype = np.float64)
    brightness_multiplier = 0.2
    random_bright = brightness_multiplier+np.random.uniform()
    img[:,:,2] = img[:,:,2]*random_bright
    img[:,:,2][img[:,:,2]>255] = 255
    img = np.array(img, dtype = np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    return img

def convert_color_image(img):
    img = cv2.cvtColor(img,cv2.COLOR_BGR2YUV)
    return img
                       
# crop bottom 25 pixels + crop the image top (scenery above road) + resize it to 64 x 64
def crop_resize_image(img):
    shape = img.shape
    img = img[math.floor(shape[0]/2.5):shape[0]-20, 0:shape[1]]
    img = cv2.resize(img, (IMAGE_W_H, IMAGE_W_H), interpolation=cv2.INTER_AREA)    
    return img

def process_image(img, angle):
    # randomly flip sampled images 
    if random() > 0.666:
        img, angle = flip_image(img, angle)
    img = modify_brightness(img)
    img = convert_color_image(img)
    img = crop_resize_image(img)
    return img, angle


def load_data(sample):
    path = base_path + '/IMG/'

    name_center = path + sample[0].split('/')[-1]
    name_left = path + sample[1].split('/')[-1]
    name_right = path + sample[2].split('/')[-1]
    
    image_center = cv2.imread(name_center)
    image_left = cv2.imread(name_left)
    image_right = cv2.imread(name_right)

    angle_center = float(sample[3])
    angle_left = angle_center + correction
    angle_right = angle_center - correction

    return (image_center, image_left, image_right), (angle_center, angle_left, angle_right)
            
def generator(samples, batch_size, total_cameras=3):
    num_samples = len(samples)
    while 1: 
        sklearn.utils.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = []
            angles = []
            for batch_sample in batch_samples:
                image, angle = load_data(batch_sample)
                for item in zip(image,angle): #iterate camera images and steering angles
                    aug_image, aug_angle = process_image(item[0], item[1])
                    if abs(aug_angle) > 0.05:
                        images.append(aug_image)
                        angles.append(aug_angle)
                
            X_train = np.array(images)
            y_train = np.array(angles)

            yield X_train, y_train

def model_comma_ai():
    model = Sequential()
    model.add(Lambda(lambda x: x/127.5 - 1., input_shape=(IMAGE_W_H, IMAGE_W_H, 3), name='Normalization'))
    model.add(Convolution2D(16, 8, 8, subsample=(4, 4), border_mode="same", activation='elu', name='conv1'))
    model.add(Convolution2D(32, 5, 5, subsample=(2, 2), border_mode="same", activation='elu', name='conv2'))
    model.add(Convolution2D(64, 5, 5, subsample=(2, 2), border_mode="same", activation='elu', name='conv3'))
    model.add(Flatten(name='flat'))
    model.add(Dropout(0.2, name='drop1'))
    model.add(ELU(name='elu1'))
    model.add(Dense(512, activation='elu', name='fully_connected1'))
    model.add(Dropout(0.5, name='drop2'))
    model.add(ELU(name='elu2'))
    model.add(Dense(1, name='output'))
    
    return model


def model_NVIDIA():
    
    drop = 0.4
    
    model = Sequential()
    model.add(Lambda(lambda x: x/127.5 - 1., input_shape=(IMAGE_W_H, IMAGE_W_H, 3), name='Normalization'))
    model.add(Convolution2D(24, 5, 5, subsample=(2,2), activation='relu', name='Conv1'))
    model.add(Convolution2D(36, 5, 5, subsample=(2,2), activation='relu', name='Conv2'))
    model.add(Convolution2D(48, 5, 5, subsample=(2,2), activation='relu', name='Conv3'))
    model.add(Convolution2D(64, 3, 3, activation='relu', name='Conv4'))
    model.add(Convolution2D(64, 3, 3, activation='relu', name='Conv5'))
    model.add(Flatten())
    model.add(Dense(1164, activation='relu', name='FC1'))
    #model.add(Dropout(drop, name='drop1'))
    model.add(Dense(100, activation='relu', name='FC2'))
    #model.add(Dropout(drop, name='drop2'))
    model.add(Dense(50, activation='relu', name='FC3'))
    #model.add(Dropout(drop, name='drop3'))
    model.add(Dense(10, activation='relu', name='FC4'))
    #model.add(Dropout(drop, name='drop4'))
    model.add(Dense(1, name='output'))
    
    return model 

def model_LeNet():
    model = Sequential()
    model.add(Lambda(lambda x: x/127.5 - 1., input_shape=(IMAGE_W_H, IMAGE_W_H, 3), name='Normalization'))
    model.add(Convolution2D(6, 5, 5, border_mode="same", activation='elu', name='Conv1'))
    model.add(MaxPooling2D())
    model.add(Convolution2D(6, 5, 5, border_mode="same", activation='elu', name='Conv2'))
    model.add(MaxPooling2D())
    model.add(Flatten())
    model.add(Dense(120, activation='elu', name='FC1'))
    model.add(Dropout(0.5, name='drop1'))
    model.add(Dense(84, activation='elu', name='FC2'))
    model.add(Dropout(0.5, name='drop2'))
    model.add(Dense(1, activation='elu', name='output'))
    model.add(Dropout(0.5, name='drop3'))

    return model

samples = []
path = base_path + '/driving_log.csv'
with open(path) as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)

from sklearn.model_selection import train_test_split
train_samples, validation_samples = train_test_split(samples, test_size=0.2)

# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size)
validation_generator = generator(validation_samples, batch_size)

iterator = generator(train_samples, batch_size)
sample_images, sample_steerings = iterator.__next__()

plt.subplots(figsize=(20, 5))
for i in range(10):
    plt.subplot(2, 5, i+1)
    plt.axis('off')
    plt.title("Steering: {:.4f}".format(sample_steerings[i]))
    plt.imshow(sample_images[i], cmap='Accent')
plt.show()


import time, os, fnmatch, shutil

def save_model(model_name):
    t = time.localtime()
    timestamp = time.strftime('%b-%d-%Y_%H-%M-%S', t)
    model_name = ('model' + model_name  + '_' + timestamp + '.h5')
    
    return model_name

from keras.models import Sequential
from keras.layers.core import Dense, Dropout, Activation
from keras.optimizers import Adam
from keras.layers import Flatten, Lambda, ELU, Cropping2D, Input, MaxPooling2D
from keras.layers.convolutional import Convolution2D
from keras.regularizers import l2

if CNN_model == "comma_ai":
    model = model_comma_ai()
else:
    if CNN_model == "LeNet":
        model = model_LeNet() 
    else:
        if CNN_model == "NVIDIA":
            model = model_NVIDIA()
                       
print("Model =", CNN_model)

model.summary()
model.compile(loss='mse', optimizer='Adam')

history_object = model.fit_generator(train_generator, 
                                     samples_per_epoch=len(train_samples)*3/batch_size,
                                     validation_data=validation_generator,
                                     nb_val_samples=len(validation_samples),
                                     nb_epoch=epoch,
                                     verbose=1)

h5_output = save_model(CNN_model) 
model.save(h5_output)
print("Model saved")
print ('batch size =', batch_size)
print ('correction', correction)
print ('epochs', epoch)

### plot the training and validation loss for each epoch
plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()


