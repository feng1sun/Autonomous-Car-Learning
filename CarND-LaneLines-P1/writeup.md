# **Finding Lane Lines on the Road** 

---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"

---

### Reflection

### 1. Describe my pipeline.

My pipeline consisted of 5 steps. First, I converted the images to grayscale, then I select yellow and white lane line. Before using canny to detect edge, I apply gaussian blur by kernel size 7.Because of camera is mounted in a fixed position, I select a fixed region in the image by using region_of_interest function. Then I apply gaussian blur again. In order to draw the lane lines, I implement a Hough Transform on edge detected image. At least, I add lane lines in the original image.

In order to draw a single line on the left and right lanes, I modified the draw_lines() function by assign all positive slope lines to the left lane while negative slope lines to the right lane. Then calculate means of left and right lane slope and intercept.


### 2. Identify potential shortcomings with my current pipeline

One potential shortcoming would be lane line detection when lane lines in shadow.

Another shortcoming could be denoise. 

### 3. Suggest possible improvements to my pipeline

A possible improvement would be to fine tune the parameters. Maybe there have another algorithms to improvement.
