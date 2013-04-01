## Triangulation Toolbox for MATLAB

_Triangulation Toolbox_ is an open-source project to share algorithms, datasets, and benchmark for landmark-based localization. It is implemented in [MATLAB][] script language, but probably working in [Octave][] and [Scilab][]. It is distributed under [Simplified BSD License][].
[MATLAB]: http://www.mathworks.com/products/matlab/
[Octave]: http://www.gnu.org/software/octave/
[Scilab]: http://www.scilab.org/
[Simplified BSD License]: http://opensource.org/licenses/BSD-2-Clause

### Installation
 1. Download the toolbox through [its GitHub ZIP URL](https://github.com/SunglokChoi/Triangulation-Toolbox/archive/master.zip)
 1. Unzip the downloaded file, _Triangulation-Toolbox-master.zip_, on your desired directory
 1. Execute `run_test_aux.m` or `run_test_localize.m` to check its working in MATLAB

### Files
 * __2D Localization Algorithms__
  * `localize2d_*.m` calculates position (and orientation) from the given landmarks and their measurements on 2D spaces.
 * __3D Localization Algorithms__
  * `localize3d_*.m` calculates position (and orientation) from the given landmarks and their measurements on 3D spaces.
 * __Observation Functions__
  * `observe_distance.m`
  * `observe_bearing.m`
  * `observe_displacement.m`
  * `observe_pose.m`
 * __Random Noise Generators__ 
  * `apply_noise_gauss.m`
 * __Accuracy Criteria__
  * `error_position.m`
  * `error_orientation.m`
 * __Utility Functions__
  * `test_is_true.m`
  * `test_is_near.m`
  * `tran_rad2deg.m`
  * `tran_deg2rad.m`
  * `tran_rad2rot.m`
  * `tran_rot2rad.m`
  * `trim_rad.m`
  * `save_figure.m`
 * __Scripts for Performance Evaluation__
  * `run_eval_random.m`
  * `run_eval_roh.m`
  * `run_anal_record.m`
 * __Scripts for Unit-test__
  * `run_test_aux.m`
  * `run_test_localize.m`
  * `run_example.m`
Use _help_ command to check input/output of each function, for example, `help observe_distance`.

### Examples
``matlab
trueMap =                                      ...
[                                              ...
    % x,  y,  z, r_x, r_y, r_z                 ...
      0,  0,  0,   0,  0,  0;                  ...
      8,  0,  0,   0,  0,  tran_deg2rad( +90); ...
      8,  8,  0,   0,  0,  tran_deg2rad(-180); ...
];
truePose = [3, 2, 0, 0, 0, pi / 9];
obsData = observe_distance(trueMap, truePose);  % Simulate observation
pose = localize2d_sayed05(obsData, trueMap);    % Estimate position
disp(pose);
``
See [`run_example.m`][] for more complex example with visualization.

[`run_example.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_example.m

### Reference
 * Sunglok Choi, __Triangulation Toolbox: Open-source Algorithms and Benchmark for Landmark-based Localization__, under review, 2013

### Acknowlegement
We appreciate the following contributors:
 * [Prof. Federico Thomas](http://www.iri.upc.edu/people/thomas/) shared his trilateration code at his homepage. ([See the algorithm](https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/localize3d_thomas05.m))
 * Hyunchul Roh provided his angulation dataset. ([See the dataset](https://github.com/SunglokChoi/Triangulation-Toolbox/tree/master/dataset_roh))

### Contact
 * [Sunglok Choi](http://sites.google.com/site/sunglok/) (sunglok AT hanmail DOT net)
