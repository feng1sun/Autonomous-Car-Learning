#**Behavioral Cloning** 
---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/model.png "Model Visualization"
[image2]: ./examples/center.png "Center image"
[image3]: ./examples/left.png "Left Image"
[image4]: ./examples/right.png "Right Image"
[image5]: ./examples/placeholder_small.png "Recovery Image"
[image6]: ./examples/image_raw.png "Normal Image"
[image7]: ./examples/image_flipped.png "Flipped Image"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup.md summarizing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

My model consists of a convolution neural network with 5x5 and 3x3 filter sizes and depths between 24 and 64 (model.py lines 105-127). All the convolution layers in this model use 2x2 stride.

After 5 convolution layers, the output is then flattened to produce a layer of 1024 neurons. The first full-connected layer then resulting in output of 1024 neurons, followed by 100 neurons, 50 neurons and 10 neurons, and finally produces an output representing the steering angle.

The model includes RELU activation to introduce nonlinearity, and the data is normalized in the model using a Keras lambda layer (code line 106). And then I includes the Cropping layer for image cropping, because not all of pixels contain useful information.

#### 2. Attempts to reduce overfitting in the model

The model contains dropout layers in order to reduce overfitting. Each convolution layer and full-connected layer followed by dropout layer with rate 0.5. However, with the experience of the training model, dropout layers between full-connected layers would cause unsafe driving. Therefore I remove the dropout layers after convolution layers.

The model was trained and validated on different data sets to ensure that the model was not overfitting (code line 93-98). I have collected my own data. Sepecially, I have notice that the distribution of steering angle in data udacity provided is not well balanced. Therefore I collect more data in large steering angle, which could make model learing more effecient. The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 129). At the end of training, the model training loss is 0.0143 and validation loss is 0.0136.

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of center lane driving, recovering from the left and right sides of the road. I collected the training data by driving the car.

For details about how I created the training data, see the next section. 

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to train a CNN to map raw pixels from camera directly to steering commands. This end-to-end approach proved surprisingly powerful.

My first step was to use a convolution neural network model similar to the Nvidia's architecture from their white paper [End to End Learning for Self-Driving Cars](http://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf) I thought this model might be appropriate because in their paper, their model proved to be very effective for self driving.

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set.

To combat the overfitting, I modified the model, add the dropout layer in each convolution and full-connected layer. this significantly improve the performance.

The final step was to run the simulator to see how well the car was driving around track one. There were a few spots where the vehicle fell off the track. to improve the driving behavior in these cases, I augment the training data, by preprocessed the image from the camera.

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.

#### 2. Final Model Architecture

The final model architecture (model.py lines 105-127) consisted of a convolution neural network with 5x5 and 3x3 filter sizes and depths between 24 and 64. All the convolution layers in this model use 2x2 stride. Then followed by 4 full-connected layers.

Here is a visualization of the architecture (note: visualizing the architecture is optional according to the project rubric)

![alt text][image1]

#### 3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded two laps on track one using center lane driving. Here is an example image of center lane driving:

![alt text][image2]

I then recorded the vehicle recovering from the left side and right sides of the road back to center so that the vehicle would learn to have better performance in left turn and right turn. These images show what a recovery looks like starting from left to right:

![alt text][image3]
![alt text][image4]

Then I repeated this process on track two in order to get more data points.

To augment the data sat, I also flipped images and angles thinking that this would improve the performance in self driving. For example, here is an image that has then been flipped:

![alt text][image6]
![alt text][image7]

After the collection process, I had 9728 number of data points. I then preprocessed this data by adding noise, adjust gamma, translate image and rotate image.

I finally randomly shuffled the data set and put 20% of the data into a validation set. 

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was 2 as evidenced by experience. I used an adam optimizer so that manually training the learning rate wasn't necessary.
