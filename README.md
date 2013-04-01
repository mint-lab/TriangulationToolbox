## Triangulation Toolbox for MATLAB

_Triangulation Toolbox_ is an open-source project to share algorithms, datasets, and benchmark for landmark-based localization. It is implemented in [MATLAB][] script language, but probably working in [Octave][] and [Scilab][]. It is distributed under [Simplified BSD License][].
[MATLAB]: http://www.mathworks.com/products/matlab/
[Octave]: http://www.gnu.org/software/octave/
[Scilab]: http://www.scilab.org/
[Simplified BSD License]: http://opensource.org/licenses/BSD-2-Clause

### Installation
 1. Download the toolbox through [its GitHub ZIP URL](https://github.com/SunglokChoi/Triangulation-Toolbox/archive/master.zip)
 1. Unzip the downloaded file, _Triangulation-Toolbox-master.zip_, on your desired directory
 1. Execute `run_test_aux` or `run_test_localize` to check its working in MATLAB

### Files
Use `help` command to check input/output of each function, for example, `help observe_distance`.
 * __2D Localization Algorithms__: `localize2d_*.m`
 * __3D Localization Algorithms__: `localize3d_*.m`
 * __Observation Functions__: [`observe_distance.m`][], [`observe_bearing.m`][], [`observe_displacement.m`][], [`observe_pose.m`][]
 * __Random Noise Generators__: [`apply_noise_gauss.m`][]
 * __Accuracy Criteria__: [`error_position.m`][], [`error_orientation.m`][]
 * __Utility Functions__
  * Unit-test: [`test_is_true.m`][], [`test_is_near.m`][]
  * Angular Conversions: [`tran_rad2deg.m`][], [`tran_deg2rad.m`][], [`tran_rad2rot.m`][], [`tran_rot2rad.m`][], [`trim_rad.m`][]
  * Miscellaneous: [`save_figure.m`][]
 * __Scripts for Examples__: [`run_example.m`][]
 * __Scripts for Performance Evaluation__
  * Benchmark with Random Landmarks ([Map Uncertainty][], [Measurement Noise][]): [`run_eval_random.m`][]
  * Benchmark with Roh's Angulation Dataset: [`run_eval_roh.m`][]
  * [Cumulative Histogram Graphs][] for Each Benchmark: [`run_anal_record.m`][]
 * __Scripts for Unit-test__: [`run_test_aux.m`][], [`run_test_localize.m`][]
 * __Datasets__
  * Roh's Angulation Dataset: [`dataset_roh`][]

[`localize3d_thomas05`]:https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/localize3d_thomas05.m
[`observe_distance.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/observe_distance.m
[`observe_bearing.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/observe_bearing.m
[`observe_displacement.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/observe_displacement.m
[`observe_pose.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/observe_pose.m
[`apply_noise_gauss.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/apply_noise_gauss.m
[`error_position.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/error_position.m
[`error_orientation.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/error_orientation.m
[`test_is_true.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/test_is_true.m
[`test_is_near.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/test_is_near.m
[`tran_rad2deg.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/tran_rad2deg.m
[`tran_deg2rad.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/tran_deg2rad.m
[`tran_rad2rot.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/tran_rad2rot.m
[`tran_rot2rad.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/tran_rot2rad.m
[`trim_rad.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/trim_rad.m
[`save_figure.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/save_figure.m
[`run_example.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_example.m
[`run_eval_random.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_eval_random.m
[`run_eval_roh.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_eval_roh.m
[`run_anal_record.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_anal_record.m
[`run_test_aux.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_test_aux.m
[`run_test_localize.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_test_localize.m
[`dataset_roh`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/dataset_roh
[Map Uncertainty]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/benchmark_result/run_eval_random(map%2C2d)/ex1_position.png
[Measurement Noise]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/benchmark_result/run_eval_random(bearing%2C2d)/ex1_position.png
[Cumulative Histogram Graphs]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/benchmark_result/run_eval_random(map%2C2d)/ex1_06_position.png

### Examples
```matlab
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
```
See [`run_example.m`][] for more complex example with visualization.

### Reference
 * Sunglok Choi, __Triangulation Toolbox: Open-source Algorithms and Benchmark for Landmark-based Localization__, under review, 2013

### Acknowlegement
 We appreciate the following contributors:
 * [Prof. Federico Thomas](http://www.iri.upc.edu/people/thomas/) shared his trilateration code at his homepage. ([See the algorithm][`localize3d_thomas05`])
 * Hyunchul Roh provided his angulation dataset. ([See the dataset][`dataset_roh`])

### Contact
 * [Sunglok Choi](http://sites.google.com/site/sunglok/) (sunglok AT hanmail DOT net)
