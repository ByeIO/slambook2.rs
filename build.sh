RUST_BACKTRACE=1 cargo run --example ch3-useEigen-linearEqSolution
RUST_BACKTRACE=1 cargo run --example ch5-imageBasics-imageBasics
RUSTFLAGS="-Znext-solver" cargo run --example ch7-triangulation
RUST_BACKTRACE=1 cargo run --example ch8-direct_method_single_layer > logs/ch8-direct_method_single_layer.log
RUST_BACKTRACE=1 cargo run --example ch8-optical_flow > logs/ch8-optical_flow.log
RUST_BACKTRACE=1 cargo run --example ch9-bundle_adjustment_g2o > logs/ch9-bundle_adjustment_g2o.log
RUST_BACKTRACE=1 cargo run --example ch10-pose_graph_g2o_SE3
RUST_BACKTRACE=1 cargo run --example ch10-pose_graph_g2o_lie_algebra > logs/ch10-pose_graph_g2o_lie_algebra
cargo run --example ch12-dense_mono-dense_mapping_image > /dev/null
cargo build --example ch12-dense_mono-dense_mapping_image
cargo run --example ch12-dense_RGBD-surfel_mapping > logs/ch12-dense_RGBD-surfel_mapping.log
cargo build --example ch13-myslam --target wasm32-wasip1-threads --release
# cargo build --example ch13-myslam --target wasm32-unknown-unknown
cargo build --example ch3-useEigen-eigenMatrix -Z unstable-options --build-plan > 构建过程.md
cargo build --example ch3-useEigen-eigenMatrix -Z unstable-options --unit-graph >> 构建过程.md
cargo build --example ch3-useEigen-eigenMatrix --target wasm32-wasip1 --release --artifact-dir ./prebuilt --keep-going

cargo build --example ch3-useEigen-eigenMatrix --keep-going
