## UTIAS MRCLAM Dataset
UTIAS Multi-Robot Cooperative Localization and Mapping Dataset (a.k.a. MRCLAM dataset) consists of 9 sets of data including odometry, distance and bearing measurements of 15 landmarks from 5 robots. The dataset originally aims cooperative localization and SLAM, but we utilize it for landmark-based localization from instantaneously available observation.

We convert MRCLAM dataset to easily apply to landmark-based localization. The instantaneously available data are extracted as observed measurements during very small motion (0.01 meters and 0.01 radians) and short time (at most 1 second). We reject the observation whose number of measurements is less than 3. Outlier measurements are also removed during conversion. Its conversion for each set is presented in [`run_conv_mrclam.m`](https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_conv_mrclam.m). The converted sets of data are written in MATLAB MAT files such as `mrclam1.mat`.

You can get more detail on MRCLAM dataset from its website and paper [1]. Please cite their paper if you use this dataset.

#### Landmark Map, Ground Truth, and Measurements
The data files, `mrclam?.mat`, contains three variables: _landmark_, _groundtruth_, and _measurement_. The variable _landmark_ contains 15 lines of information with 3 columns which sequentially mean landmark ID and 2D position. In contrast to the original dataset, the landmark IDs are rearranged to begin with 1. The variable _groundtruth_ contains 4 columns of data which are experiment ID and true 2D pose (position and orientation) of the robot. The experiment IDs start from 1 and its number is same with the number of lines. The variable _measurement_ includes 4 columns of data which represent the experiment ID, observed landmark ID, and its range and bearing measurements. Observations with same experiment ID mean a single set of experiments. Units of all values are meter for distance and radian for angle. In overall, three variables contains the following.
 * _landmark_: landmark ID, position X, position Y (of each landmark)
 * _groundtruth_: experiment ID, position X, position Y, orientation (of each robot's true pose)
 * _measurement_: experiment ID, observed landmark ID, observed distance, observed bearing (from each robot's true pose to a landmark)

#### Reference
 * [1] K. Leung, Y. Halpern, T. Barfoot, and H. Liu, __The UTIAS Multi-Robot Cooperative Localization and Mapping Dataset__, The International Journal of Robotics Research, Vol. 30, No. 8, 2011 [IJRR Online](http://ijr.sagepub.com/content/30/8/969) [Website](http://asrl.utias.utoronto.ca/datasets/mrclam/)
