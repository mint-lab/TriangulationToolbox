## Roh's Angulation Dataset

_Roh's angulation dataset_ is a set of measurements acquired from a rotating IR detector and four IRED landmarks. The sensor system aims at 2D bearing-based localization, which is described in Roh's thesis [1] and paper [2] in detail.

#### Landmark Map
The map file, _landmark.csv_, contains position of four landmarks.
 * Landmark #1: (0, 0, 0) meters
 * Landmark #2: (6, 0, 0) meters
 * Landmark #3: (6, 6, 0) meters
 * Landmark #4: (0, 6, 0) meters

#### Measurements and Their Ground Truth
The data files, _(Xx.x,Yy.y).csv_, contain four bearing measurements from four landmarks at the given true position. The true position is presented in each filename, for example, measurements in _(X3.0,Y1.5).csv_ is acquired at (3.0, 1.5, 0.0) meters with orientation of (0.0, 0.0, pi/2) degrees. Each file contains 200 sets of measurements whose each column is a bearing angle (in degrees) from each landmark.
 * Column #1: a bearing angle (in degrees) from landmark #1
 * Column #2: a bearing angle (in degrees) from landmark #2
 * Column #3: a bearing angle (in degrees) from landmark #3
 * Column #4: a bearing angle (in degrees) from landmark #4

#### Reference
 * [1] H. C. Roh, __Mobile Robot Localization based on Active Beacon System using Infrared Sensor in Indoor Environment__, KAIST, Robotics Program, Master's Thesis, 2010 [KAIST Library](http://library.kaist.ac.kr/thesis02/2010/2010M020093169_S1Ver2.pdf)
 * [2] H. C. Roh, Y. G. Ryu, and M. J. Chung, __Mobile Robot Localization based on Active Beacon System using Infrared Sensors in Indoor Environment__, Korea Robotics Society Annual Conference (KRoC), 2010
