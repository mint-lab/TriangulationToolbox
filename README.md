## Triangulation Toolbox for MATLAB

_Triangulation Toolbox_ is an open-source project to share algorithms, datasets, and benchmarks for landmark-based localization. It is implemented in [MATLAB][] script language and distributed under [Simplified BSD License][].
 * Homepage: [http://sites.google.com/site/sunglok/tt](http://sites.google.com/site/sunglok/tt)

### Installation
 1. Download the toolbox through [its GitHub ZIP URL](https://github.com/SunglokChoi/Triangulation-Toolbox/archive/master.zip)
 1. Unzip the downloaded file, _Triangulation-Toolbox-master.zip_, on your target directory
 1. Execute `run_test_aux` or `run_test_localize` to check its working in MATLAB

### File Description
Use `help` command to know each function in detail, for example, `help observe_distance`.
 * __2D Localization Algorithms__: `localize2d_*.m`
 * __3D Localization Algorithms__: `localize3d_*.m`
 * __Observation Functions__: [`observe_distance.m`][], [`observe_bearing.m`][], [`observe_displacement.m`][], [`observe_pose.m`][]
 * __Random Noise Generators__: [`apply_noise_gauss.m`][]
 * __Accuracy Criteria__: [`error_position.m`][], [`error_orientation.m`][]
 * __Utility Functions__
  * Unit-test: [`test_is_true.m`][], [`test_is_near.m`][]
  * Angular Conversions: [`tran_rad2deg.m`][], [`tran_deg2rad.m`][], [`tran_rad2rot.m`][], [`tran_rot2rad.m`][], [`trim_rad.m`][]
 * __Scripts for Examples__: [`run_example.m`][]
 * __Scripts for Unit-test__: [`run_test_aux.m`][], [`run_test_localize.m`][]
 * __Scripts for Performance Evaluation__
  * Benchmark with Random Landmarks (with Map Uncertainty and Measurement Noise): [`run_eval_random.m`][]
  * Benchmark with Roh's Angulation Dataset: [`run_eval_roh.m`][]
  * Benchmark with MRCLAM Dataset: [`run_eval_mrclam.m`][]
  * [Position/Orientaion Estimate Distribution Drawing][] for Each Benchmark: [`run_draw_distribution.m`][]
 * __Real Datasets__
  * Roh's Angulation Dataset: [`dataset_roh`][]
  * MRCLAM Dataset: [`dataset_mrclam`][] (conversed by [`run_conv_mrclam.m`][])

### Example
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
estPose = localize2d_sayed05(obsData, trueMap); % Estimate position
```
See [`run_example.m`][] for more complex example with visualization.

### Reference
 * Sunglok Choi, __Triangulation Toolbox: Open-source Algorithms and Benchmark for Landmark-based Localization__, under review, 2013

### Acknowledgement
 We appreciate the following contributors:
 * [Prof. Federico Thomas](http://www.iri.upc.edu/people/thomas/) shared his trilateration code at his homepage. (See [the algorithm][`localize3d_thomas05.m`])
 * Hyunchul Roh provided his angulation dataset. (See [the dataset][`dataset_roh`])
 * [Dr. Keith Leung](http://asrl.utias.utoronto.ca/~kykleung), Yoni Halpern, [Prof. Tim Barfoot](http://asrl.utias.utoronto.ca/~tdb), and [Prof. Hugh Liu](http://www.flight.utias.utoronto.ca/fsc/index.php?id=204) shared their MRCLAM dataset. (See [the dataset][`dataset_mrclam`])

### Contact
 * [Sunglok Choi](http://sites.google.com/site/sunglok/) (sunglok AT hanmail DOT net)

[MATLAB]: http://www.mathworks.com/products/matlab/
[Simplified BSD License]: http://opensource.org/licenses/BSD-2-Clause
[`localize3d_thomas05.m`]:https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/localize3d_thomas05.m
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
[`run_example.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_example.m
[`run_test_aux.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_test_aux.m
[`run_test_localize.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_test_localize.m
[`run_eval_random.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_eval_random.m
[`run_eval_roh.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_eval_roh.m
[`run_eval_mrclam.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_eval_mrclam.m
[`run_draw_distribution.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_draw_distribution.m
[`dataset_roh`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/dataset_roh
[`dataset_mrclam`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/dataset_mrclam
[`run_conv_mrclam.m`]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/run_conv_mrclam.m
[Position/Orientaion Estimate Distribution Drawing]: https://github.com/SunglokChoi/Triangulation-Toolbox/blob/master/benchmark_result/run_eval_random(map%2C2d)/ex1_06_position.png
