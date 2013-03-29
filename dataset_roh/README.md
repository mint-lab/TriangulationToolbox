### Roh's Angulation Dataset

_Roh's angulation dataset_ is a set of measurements acquired from an IR detector and four IRED landmark. The sensor system aims at 2D bearing-based localization which is described in Roh's thesis [1] and paper [2] in detail.

#### Landmark Map
The map file, _landmark.csv'_, contains position of four landmarks.
 * Landmark #1: (0, 0, 0) meters
 * Landmark #2: (0, 6, 0) meters
 * Landmark #3: (6, 0, 0) meters
 * Landmark #4: (6, 6, 0) meters

#### Measurements and Their Ground Truth
The data files, _(X?.?,Y?.?,Z?.?).csv_, contains four bearing measurements from four landmarks at the given true position.
The true position is presented in each filename, for example, _(X3.0,Y1.5,Z0.0).csv_ is retrieved at (3.0, 1.5, 0.0) meters.
Each file contains 200 sets of measurements whose each column is a bearing angle (in degree) from each landmark.

#### Reference
[1] H. C. Roh, __Mobile Robot Localization based on Active Beacon System using Infrared Sensor in Indoor Environment__, KAIST, Robotics Program, Master's Thesis, 2010 [KAIST Library](http://library.kaist.ac.kr/thesis02/2010/2010M020093169_S1Ver2.pdf)
[2] H. C. Roh, Y. G. Ryu, and M. J. Chung, __Mobile Robot Localization based on Active Beacon System using Infrared Sensors in Indoor Environment__, Korea Robotics Society Annual Conference (KRoC), 2010
