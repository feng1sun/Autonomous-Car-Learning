## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./examples/undistort_output.png "Undistorted"
[image2]: ./emamples/undistorted_image.png "Road Transformed"
[image3]: ./examples/binary_image.png "Binary Example"
[image4]: ./examples/warp_image.png "Warp Example"
[image5]: ./examples/RANSAC.jpg "RANSAC denoise"
[image6]: ./examples/fit_line.png "lane line"
[image7]: ./examples/out_img.jpg "output"
[video1]: ./project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  


---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.

The code for this step is contained in the third code cell of the IPython notebook located in "CarND-Advanced-Lane-Lines.ipynb"

I start by preparing "object points", which will be the (x, y, z) coordinates of the chessboard corners in the world. Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

Using undistortion matrix obtained at the calibration step, I execute `cv2.undistort` function to undo the distortion. You can see result below:
![alt text][image2]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

Obtaining good quality edges masks is a key step to perform an accurate lane detection. I used a combination of color and gradient thresholds to generate a binary image. I’ve observed that detection works better on S channel of HLS on some set of images, while on others L works better. So I used OR combination to get both. Here's an example of my output for this step. 

![alt text][image3]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The code for my perspective transform includes a function called `warpe_image()`.  The `warpe_image()` function takes as inputs an image (`img`) and transform matrix (`mtx`). The function called `perspective_transform` is using to calculate the perspective transform matrix. The source and destination points in the following:

```python
src_pts = np.array([[240,720],[575,460],[715,460],[1150,720]], np.float32)
dst_pts = np.array([[440,720],[440,0],[950,0],[950,720]], np.float32)
```

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 240, 720      | 440, 720      | 
| 575, 460      | 440, 0        |
| 715, 460      | 950, 0        |
| 1150, 720     | 950, 720      |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

I use find_centroids function to find the center in each search window, and return window_boole to identify which window is reasonable. After findding centroids i used find_xy function to fetch points in each reasonable window, and then use line_fit function to fit the lane line. However, the noise in warped image would significantly influence the fitting. Therefore I used RANSAC algorithm to denoise the warped image before fitting lane line. The result of RANSAC is shown below.

![alt text][image5]

as shown above, the green point will be used to fit the lane line. The blue point is noise. After used RANSAC to fetch the reasonable point, I used `np.polyfit` to fit the lane line. And then I draw the line in the image by using `cv2.fillPoly` function. The result is shown below:

![alt text][image6]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

First step is to obtain polynomial for the lane lines in meters, not in pixels. For that, knowing the US government requirements for lane width and dashed line distance, I estimated the conversion coefficients and performed a fit. Second step is to find the line in between of two lane lines. This is done through evaluating polynomial fits for each lane at every vertical position, and finding the center. Third step is to find polynomial fit for the center line. Finally, I use first and second derivative to obtain a curvature.

To obtain center offset I evaluate X position of left and right lane polynomials, and find point in the middle (in meters). Then I find the distance from the center of the image (in meters) to that middle of the lane position. That distance is the center offset.

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

Here is an example of my result on a test image:

![alt text][image7]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?


I've tried more complicate approach to find the lane line. However one approach might work well in one situation, but not well in another situation. The key of this problem is edge mask. a good quality edge mask is a key step to perform an accurate lane detection. Therefore I apply RANSAC algorithm to denoise before fit the lane line. It make more robust.

As the result shown, The pipeline performs well on first two videos. The performance on the third very challenging video is clearly not as good. I think the current implementation can not well detect lane line in bad image brightness and image contrast. therefore, It's necessary to process image before detect lane line. I have tried to using cv2.equalizeHist function to equalize the histogram in 3 color channels. However It dose not perform well. 

I have also tried to add some logical element.  For example,  two lane line should have roughtly same curvature, and two lane line should have same direction, and two lane line should have fixed distance. etc. however i found that if I add too much logical detection, the lane line finding system would not perform too much better. Therefor I give up this approach.

























