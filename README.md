# slambook2的rust重构
很多bug🕶️, 有些实现可能比较简略😂.
已经将算法全用rust重写了.

## 代码目录
- examples文件夹
- src文件夹
```sh
cargo run --example ch3-coordinateTransform
cargo run --example ch3-useEigen-eigenMatrix
cargo run --example ch3-useEigen-linearEqSolution
cargo run --example ch3-useEigen-matrixExtractAssign
cargo run --example ch3-useGeometry
cargo run --example ch3-visualizeGeometry
cargo run --example ch3-plotTrajectory
cargo run --example ch4-trajectoryError
cargo run --example ch4-useSophus
cargo run --example ch4-useSophus_factrs
cargo run --example ch5-imageBasics-imageBasics
cargo run --example ch5-imageBasics-undistortImage
cargo run --example ch5-rgbd-joinMap
cargo run --example ch5-stereo-stereoVision
cargo run --example ch6-ceresCurveFitting
cargo run --example ch6-gaussNewton
cargo run --example ch6-g2oCurveFitting
cargo run --example ch7-orb_cv
cargo run --example ch7-orb_self
cargo run --example ch7-pose_estimation_2d2d
cargo run --example ch7-pose_estimation_3d2d
cargo run --example ch7-pose_estimation_3d3d
cargo run --example ch7-triangulation
cargo run --example ch8-direct_method_multi_layer
cargo run --example ch8-direct_method_single_layer
cargo run --example ch8-optical_flow
cargo run --example ch9-bundle_adjustment_ceres
cargo run --example ch9-bundle_adjustment_g2o
cargo run --example ch10-pose_graph_g2o_SE3
cargo run --example ch10-pose_graph_g2o_gs_rs
cargo run --example ch10-pose_graph_g2o_lie_algebra
cargo run --example ch11-feature_training
cargo run --example ch11-gen_vocab_large
cargo run --example ch11-loop_closure
cargo run --example ch12-dense_RGBD-octomap_mapping
cargo run --example ch12-dense_RGBD-pointcloud_mapping
cargo run --example ch12-dense_RGBD-surfel_mapping
cargo run --example ch12-dense_mono-dense_mapping
cargo run --example ch12-dense_mono-dense_mapping_image
cargo run --example ch13-myslam
```

## 编译
### 编译器版本
测试编译通过的编译器:
```sh
rustc --version; rustup --version; cargo --version
>rustc 1.83.0-nightly (14f303bc1 2024-10-04)
>rustup 1.27.1 (54dd3d00f 2024-04-24)
>info: This is the version for the rustup toolchain manager, not the rustc compiler.
>info: The currently active `rustc` version is `rustc 1.83.0-nightly (14f303bc1 2024-10-04)`
>cargo 1.83.0-nightly (80d82ca22 2024-09-27)
```

### 使用本地crate
库文件在vendor文件夹.
1. 将库文件离线到本地:
```sh
cargo vendor
```
2. 使用本地库文件
**.cargo/config.toml**
```toml
[source.crates-io]
replace-with = "vendored-sources"

[source.vendored-sources]
directory = "vendor"
```
3. 如果遇到编译时有关库文件的报错直接修改vendor文件夹内的库文件即可.
