# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.



---

### Data Buffer

The data buffer was implemented using a vector, who will erase the first (the oldest) element once its size exceeds 2

### Keypoints

All the different types of detectors were implemented in `matching2D_Student.cpp` in the methods `detKeypointsShiTomasi()`, `detKeypointsHarris()`, and `detKeypointsModern()`. The switching between different modern detectors in `detKeypointsModern()` were done by selecting based on the `detectorType` keyword.

The ROI was defined by a `cv::Rect vehicleRect` and it provides a function `contains()` to exam if a point is inside the region.

### Descriptors

All the different types of descriptors were stored in `descriptorTypeList` and for each detector it will loop through all the possible descriptors to try out every combination. For each combination, a total of 10 epochs on 10 images were run and the average performance of this combination were recorded and printed by the end of the testing.



## Performance Evaluation 

All of the original test results can be found in the `results.ods` file.

Here are only the summaries and conclusions of the test results.

* All results are average values tested on 10 images over 10 epochs

  

#### 1. Key-point Detection

| Detector  | Number of Key-points | Time Taken | Time per 1k Key-points | Multiple   | Distribution of Neighborhood Size  |
| --------- | -------------------- | ---------- | ---------------------- | ---------- | ---------------------------------- |
| SHITOMASI | 1339                 | 8.7 ms     | 5.9 ms                 | 6.9x       | very small                         |
| HARRIS    | 516                  | 8.2 ms     | 15.7 ms                | 2.6x       | very small                         |
| FAST      | **4905**             | **1.7 ms** | **0.4 ms**             | **114.8x** | very small                         |
| BRISK     | 2707                 | 27 ms      | 9.6 ms                 | 4.2x       | **vary from medium to very large** |
| ORB       | 500                  | 4.9 ms     | 9.9 ms                 | 4.1x       | **vary from medium to very large** |
| AKAZE     | 1342                 | 40.9 ms    | 30.5 ms                | 1.3x       | vary from small to medium          |
| SIFT      | 1380                 | 53.7 ms    | 40.6 ms                | 1.0x       | vary from small to large           |



#### 2. Number of Matched Key-points

| Detector  | Number of Key-points | Number of Key-points in the ROI | BRISK | BRIEF | ORB  | FREAK | AKAZE | SIFT |
| --------- | -------------------- | ------------------------------- | ----- | ----- | ---- | ----- | ----- | ---- |
| SHITOMASI | 1339                 | 118                             | 118   | 118   | 118  | 118   | /     | 118  |
| HARRIS    | 516                  | 56                              | 55    | 55    | 55   | 55    | /     | 55   |
| FAST      | **4905**             | **410**                         | 410   | 410   | 410  | 410   | /     | 410  |
| BRISK     | 2707                 | 273                             | 273   | 273   | 273  | 253   | /     | 273  |
| ORB       | 500                  | 115                             | 104   | 113   | 113  | 60    | /     | 113  |
| AKAZE     | 1342                 | 164                             | 164   | 164   | 164  | 164   | 164   | 164  |
| SIFT      | 1380                 | 137                             | 137   | 137   | /    | 136   | /     | 137  |



#### 3. Total Time Taken (ms)

| Detector  | Number of Key-points | Number of Key-points in the ROI | BRISK   | BRIEF   | ORB     | FREAK | AKAZE | SIFT |
| --------- | -------------------- | ------------------------------- | ------- | ------- | ------- | ----- | ----- | ---- |
| SHITOMASI | 1339                 | 118                             | 10.0    | 9.5     | 101.5   | 33.5  | /     | 17.4 |
| HARRIS    | 516                  | 56                              | 9.1     | 8.9     | 10.6    | 32.5  | /     | 16.7 |
| FAST      | **4905**             | **410**                         | **4.5** | **2.8** | **4.5** | 28.1  | /     | 12.7 |
| BRISK     | 2707                 | 273                             | 29.4    | 28.2    | 36.2    | 53.8  | /     | 43.7 |
| ORB       | 500                  | 115                             | 6.0     | 5.4     | 13.3    | 29.2  | /     | 22.3 |
| AKAZE     | 1342                 | 164                             | 41.3    | 40.7    | 46.2    | 65.2  | 72.1  | 52.6 |
| SIFT      | 1380                 | 137                             | 57.4    | 57.0    | /       | 80.9  | /     | 99.3 |

Based on the above table, the top 3 combinations recommended are:

1. **FAST + BRIEF**
2. **FAST + BRISK**
3. **FAST + ORB**