use gs_rs::optimizer::optimize;
use gs_rs::parser::g2o::G2oParser;
use gs_rs::parser::Parser;

fn main() {
    // 解析包含3D变量和里程计的g2o文件为内部因子图表示
    let factor_graph = G2oParser::parse_file("./assets/ch10-sphere.g2o").unwrap();

    // 使用10次迭代优化因子图的变量
    optimize(&factor_graph, 10);

    // 生成包含优化后3D变量和未改变里程计的g2o文件
    G2oParser::compose_file(&factor_graph, "./ch10-pose_graph_g2o_gs_rs-sphere-optimized.g2o").unwrap();
}