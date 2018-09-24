**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./examples/car_not_car.png
[image2]: ./examples/HOG_example.jpg
[image3]: ./examples/sliding_window.png
[image4]: ./examples/sliding_window.jpg
[image5]: ./examples/bboxes_and_heat.png
[image6]: ./examples/labels_map.png
[image7]: ./examples/output_bboxes.png
[video1]: ./project_video.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Vehicle-Detection/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for this step is contained in Feature extraction function which I have made annotation in my notebook. In the `extract_features` function, I get the image spatial features, HOG features and color histogram by calling `bin_spatial`, `hog_features` and `color_hist` function respectively. 

I found that if I use **YCrCb** color space will significantly improve the classifier accuracy. I set the HOG parameters of orientations=9, pixels_per_cell=(8, 8) and cells_per_block=(2, 2).

I use StandardScaler to normalize the image features before training the SVM classifier. For the convenient and memery effcient, I training the StandardScaler and save the model before using. 

I started by reading in all the `vehicle` and `non-vehicle` images.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

![alt text][image1]

#### 2. Explain how you settled on your final choice of HOG parameters.

I tried various combinations of parameters and finally I chose YCrCb color space, orientations=9, pixels_per_cell=(8, 8) and cells_per_block=(2, 2) with 3 color channels RGB in HOG features. SVM classifier test accuracy is about 99.6%. After training I saved the SVM classifier. 
 
#### 3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

I trained a linear SVM by using `grid_search.GridSearchCV` function to find good SVM parameters.

The `grid_search.GridSearchCV` return:

```python
GridSearchCV(cv=None, error_score='raise',
       estimator=SVC(C=1.0, cache_size=200, class_weight=None, coef0=0.0,
  decision_function_shape=None, degree=3, gamma='auto', kernel='rbf',
  max_iter=-1, probability=False, random_state=None, shrinking=True,
  tol=0.001, verbose=False),
       fit_params={}, iid=True, n_jobs=1,
       param_grid={'C': [1, 10], 'kernel': ('linear', 'rbf')},
       pre_dispatch='2*n_jobs', refit=True, scoring=None, verbose=0)
```

I used `get_params()` method to get the parameters.

```python
{'cv': None,
 'error_score': 'raise',
 'estimator': SVC(C=1.0, cache_size=200, class_weight=None, coef0=0.0,
   decision_function_shape=None, degree=3, gamma='auto', kernel='rbf',
   max_iter=-1, probability=False, random_state=None, shrinking=True,
   tol=0.001, verbose=False),
 'estimator__C': 1.0,
 'estimator__cache_size': 200,
 'estimator__class_weight': None,
 'estimator__coef0': 0.0,
 'estimator__decision_function_shape': None,
 'estimator__degree': 3,
 'estimator__gamma': 'auto',
 'estimator__kernel': 'rbf',
 'estimator__max_iter': -1,
 'estimator__probability': False,
 'estimator__random_state': None,
 'estimator__shrinking': True,
 'estimator__tol': 0.001,
 'estimator__verbose': False,
 'fit_params': {},
 'iid': True,
 'n_jobs': 1,
 'param_grid': {'C': [1, 10], 'kernel': ('linear', 'rbf')},
 'pre_dispatch': '2*n_jobs',
 'refit': True,
 'scoring': None,
 'verbose': 0}
```

The best parameters are following:

```python
{'C': 10, 'kernel': 'rbf'}
```
The test accuracy is 0.99577702702702697

### Sliding Window Search

#### 1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

I  segmented the image into 6 partially overlapping zones with different sliding window sizes to account for different distances.
The window sizes are  (80,50),(100,100),(200,150),(200,200),(300,230) and (300,256) pixels for each zone. Within each zone adjacent windows have an ovelap of 90%, as illustrated below. The search over all zones is implemented in the `detection(image)` function. 

![alt text][image3]

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

I am limiting my search to the right half of the frame. This trick detections in my video and allowed for faster iterations. In the real world, It should search whole image.

I perform a simple sliding window search to extract cropped images for classification. I searched on two scales using YCrCb 3-channel HOG features plus spatially binned color and histograms of color in the feature vector, which provided a nice result.

And I use two different threshold to filter the heat image. this trick allow search algorithm perform more robust.


---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](.videos_output/project_video_output.mp4)


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I created a class which can also find the lane line and calculate the curvature of lane line. The class is come from last project of advanced lane line finding. I extend the class to detect the vehicle. From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  I used different threshold to filter false positive.  For closer vehicels, I use bigger threshold. I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.

Here's an example result showing the heatmap from a series of frames of video, the result of `scipy.ndimage.measurements.label()` and the bounding boxes then overlaid on the last frame of video:


### Here are six frames and their corresponding heatmaps:

![alt text][image5]

### Here is the output of `scipy.ndimage.measurements.label()` on the integrated heatmap from all six frames:
![alt text][image6]

### Here the resulting bounding boxes are drawn onto the last frame in the series:
![alt text][image7]



---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.


**First**, I preprocessed the taining data. I extracted HOG feature, color feature from an image. After extracted features, I trained a SVM classifier. I used `grid_search.GridSearchCV` function to find good SVM parameters. The accuracy of classifier is roughly 99.5%. Then I saved the SVM model.

**Second**, I implemented a search window approach. I searched 6 different window size in a frame. Different size for different vehicle position searching. I used 2 different threshold to filtered false positive.

**Finally**, I created a class to process each frame. However the original approach was very time consumption. A way to improve speed would be to compute the HOG features only once for the entire region of interest and then select the right feature vectors, when the image is slid across. 


**What could you do to make it more robust?**

Some false positives still remain after heatmap filtering. This may be improvable by using more labeled data. Using Deep learning algorithm such as YOLO may improve the detection system.
