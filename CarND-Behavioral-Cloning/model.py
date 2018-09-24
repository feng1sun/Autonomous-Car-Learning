%matplotlib inline
import matplotlib.pyplot as plt
import csv
import numpy as np
import cv2

lines = []
with open('./train_data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        lines.append(line)
        
def img_add_noise(img, sigma=10):
    mean = np.mean(img)
    noise = sigma*np.random.randn(img.shape[0],img.shape[1],img.shape[2])+mean
    img = img + noise
    img = (img-np.min(img))/(np.max(img)-np.min(img))
    return img

def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)

def translate_img(img):
    y,x,c = img.shape
    translation_x = np.random.uniform(-0.2*x,0.2*x)
    translation_matrix = np.float32([[1,0,translation_x],[0,1,0]])
    translation__img = cv2.warpAffine(img,translation_matrix,(x,y))
    return translation__img

def rotate_img(img):
    y,x,c = img.shape
    angle = np.random.uniform(-10,10)
    scale = 1
    rotation_point = (x/2,y/2)
    rotation_matrix = cv2.getRotationMatrix2D(rotation_point,angle,scale)
    rotated_img = cv2.warpAffine(img,rotation_matrix,(x,y))
    return rotated_img

from sklearn.utils import shuffle

def generator(sample,batch_size=16):
    num_sample = len(sample)
    while 1:
        shuffle(sample)
        for offset in range(0, num_sample, batch_size):
            batch = sample[offset:offset+batch_size]
            images = []
            measurements = []
            for line in batch:
                for i in range(3):
                    image_name = line[i].split('/')[-1]
                    image_path = './train_data/IMG/' + image_name
                    image = cv2.imread(image_path)
                    img_flipped = cv2.flip(image,1)
                    img_rotated = rotate_img(image)
                    img_blur = cv2.GaussianBlur(image,(7,7),0)
                    images.append(image)
                    images.append(img_flipped)
                    images.append(img_rotated)
                    images.append(img_blur)
                    camera = image_name.split('_')[0]
                    steering = float(line[3])
                    if(camera == 'center'):
                        steering_flipped_center = -steering
                        measurements.append(steering) # image center
                        measurements.append(steering_flipped_center) # steering_flipped center
                        measurements.append(steering) # img_rotated center
                        measurements.append(steering) # img_blur center
                    elif(camera == 'left'):
                        correction = 0.15
                        steering_left = steering + correction
                        steering_flipped_left = -steering_left
                        measurements.append(steering_left) # image left
                        measurements.append(steering_flipped_left) # steering_flipped left
                        measurements.append(steering_left) # img_rotated left
                        measurements.append(steering_left) # img_blur left
                    elif(camera == 'right'):
                        correction = 0.15
                        steering_right = steering - correction
                        steering_flipped_right = -steering_right
                        measurements.append(steering_right) # image right
                        measurements.append(steering_flipped_right) # steering_flipped right
                        measurements.append(steering_right) # img_rotated right
                        measurements.append(steering_right) # img_blur right
                        
            x = np.array(images)
            y = np.array(measurements)
            yield shuffle(x,y)
            
from sklearn.model_selection import train_test_split
train_samples, validation_samples = train_test_split(lines, test_size=0.2)

batch_size = 16
train_gen = generator(train_samples, batch_size)
valid_gen = generator(validation_samples, batch_size)

import keras
from keras.models import Sequential
from keras.layers import Dense, Flatten, Conv2D, MaxPooling2D, Dropout, Lambda, Cropping2D, BatchNormalization
from keras import regularizers

model = Sequential()
model.add(Lambda(lambda x:x/255.0-0.5, input_shape=(160, 320, 3)))
model.add(Cropping2D(cropping=((40,10),(0,0))))
model.add(Conv2D(24, (5, 5), strides = (2,2), padding='valid', use_bias=True,activation='relu'))
model.add(Dropout(0.5))
model.add(Conv2D(36, (5, 5), strides = (2,2), padding='valid', use_bias=True, activation='relu'))
model.add(Dropout(0.5))
model.add(Conv2D(48, (5, 5), strides = (2,2), padding='valid', use_bias=True, activation='relu'))
model.add(Dropout(0.5))
model.add(Conv2D(64, (3, 3), strides = (2,2), padding='valid', use_bias=True, activation='relu'))
model.add(Dropout(0.5))
model.add(Conv2D(64, (3, 3), strides = (2,2), padding='valid', use_bias=True, activation='relu'))
model.add(Dropout(0.5))
model.add(Flatten())
model.add(Dense(1024))
model.add(Dense(100))
model.add(Dense(50))
model.add(Dense(10))
model.add(Dense(1))

model.compile(loss='mse', optimizer='adam')
# , initial_epoch = 10
model.fit_generator(generator=train_gen, steps_per_epoch=len(train_samples)/batch_size, epochs=3, validation_data=valid_gen, 
                    validation_steps=len(validation_samples)/batch_size, initial_epoch = 0)

model.save('model.h5')

from keras.utils.visualize_util import plot
plot(model, to_file='model.png', show_shapes=True)
