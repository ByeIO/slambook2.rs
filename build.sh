RUST_BACKTRACE=1 cargo run --example ch3-useEigen-linearEqSolution
RUST_BACKTRACE=1 cargo run --example ch5-imageBasics-imageBasics
RUSTFLAGS="-Znext-solver" cargo run --example ch7-triangulation
RUST_BACKTRACE=1 cargo run --example ch8-direct_method_single_layer > logs/ch8-direct_method_single_layer.log
RUST_BACKTRACE=1 cargo run --example ch8-optical_flow > logs/ch8-optical_flow.log
RUST_BACKTRACE=1 cargo run --example ch9-bundle_adjustment_g2o > logs/ch9-bundle_adjustment_g2o.log
RUST_BACKTRACE=1 cargo run --example ch10-pose_graph_g2o_SE3
RUST_BACKTRACE=1 cargo run --example ch10-pose_graph_g2o_lie_algebra > logs/ch10-pose_graph_g2o_lie_algebra