#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(non_fmt_panics)]
#![allow(unused_mut)]
#![allow(unused_assignments)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
#![allow(rustdoc::missing_crate_level_docs)]
#![allow(unsafe_code)]
#![allow(clippy::undocumented_unsafe_blocks)]
#![allow(unused_must_use)]
#![allow(non_snake_case)]

/*
 * 本程序演示如何用factrs solver进行位姿图优化
 * sphere.g2o是人工生成的一个Pose graph，我们来优化它。
 * 尽管可以直接通过load函数读取整个图，但我们还是自己来实现读取代码，以期获得更深刻的理解
 * 这里使用factrs::variables中的SE3表示位姿，它实质上是四元数而非李代数.
*/

use std::fs::File;
use std::io::{self, BufRead, Write, BufReader};
use std::path::Path;

// 线性代数
use nalgebra::{DMatrix, DVector, Matrix3, Vector3, SVector, Matrix, Vector6, Matrix6};

// 图优化
use factrs::{
    assign_symbols,
    core::{BetweenResidual, GaussNewton, Graph, Values},
    dtype, fac,
    linalg::{Const, ForwardProp, Numeric, NumericalDiff, VectorX, DiffResult, MatrixX},
    residuals::{Residual1, Residual2},
    traits::*,
    variables::{VectorVar2, SE2, VectorVar3, SE3, SO3, SO2},
    containers::Key,
    noise::{GaussianNoise},
    optimizers::{LevenMarquardt}
};

// 定义符号变量
assign_symbols!(X: SE3);

fn main() -> io::Result<()> {
    let file_path = "./assets/ch10-sphere.g2o";
    let file = File::open(file_path)?;
    let reader = io::BufReader::new(file);

    let mut graph = Graph::new();
    let mut values = Values::new();

    let mut vertex_cnt = 0;
    let mut edge_cnt = 0;

    for line in reader.lines() {
        let line = line?;
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        match parts[0] {
            // SE3 顶点
            "VERTEX_SE3:QUAT" => {
                let index = parts[1].parse::<usize>().unwrap();
                let x = parts[2].parse::<f64>().unwrap();
                let y = parts[3].parse::<f64>().unwrap();
                let z = parts[4].parse::<f64>().unwrap();
                let qx = parts[5].parse::<f64>().unwrap();
                let qy = parts[6].parse::<f64>().unwrap();
                let qz = parts[7].parse::<f64>().unwrap();
                let qw = parts[8].parse::<f64>().unwrap();

                let rotation = SO3::from_xyzw(qx, qy, qz, qw);
                let translation = Vector3::new(x, y, z);
                let se3 = SE3::from_rot_trans(rotation, translation);
                values.insert(X(index.try_into().unwrap()), se3);
                vertex_cnt += 1;
            }
            // SE3-SE3 边
            "EDGE_SE3:QUAT" => {
                let idx1 = parts[1].parse::<usize>().unwrap();
                let idx2 = parts[2].parse::<usize>().unwrap();

                let dx = parts[3].parse::<f64>().unwrap();
                let dy = parts[4].parse::<f64>().unwrap();
                let dz = parts[5].parse::<f64>().unwrap();
                let dqx = parts[6].parse::<f64>().unwrap();
                let dqy = parts[7].parse::<f64>().unwrap();
                let dqz = parts[8].parse::<f64>().unwrap();
                let dqw = parts[9].parse::<f64>().unwrap();

                // let info_matrix = Matrix6::from_fn(|i, j| parts[10 + i * 6 + j].parse::<f64>().unwrap());

                let rotation = SO3::from_xyzw(dqx, dqy, dqz, dqw);
                let translation = Vector3::new(dx, dy, dz);
                let delta = SE3::from_rot_trans(rotation, translation);

                // 构建双变量约束关系的因子图
                let residual = BetweenResidual::new(delta);
                // let noise = GaussianNoise::<6>::from_scalar_sigma(info_matrix);
                let noise = GaussianNoise::<6>::identity();
                graph.add_factor(fac![residual, (X(idx1.try_into().unwrap()), X(idx2.try_into().unwrap())), noise]);
                edge_cnt += 1;
            }
            _ => {}
        }
    }

    println!("read total {} vertices, {} edges.", vertex_cnt, edge_cnt);

    println!("optimizing ...");
    
    /*
    当 Cholesky 分解失败时，通常是因为矩阵不是正定的。这在优化问题中可能发生，尤其是当 Hessian 矩阵（即二阶导数矩阵）不是正定时。Cholesky 分解要求矩阵必须是正定的，因此使用基于 Cholesky 分解的优化器（如 Gauss-Newton）可能会失败。
    为了解决这个问题，可以使用 Levenberg-Marquardt 优化器。该优化器通过引入阻尼因子来处理非正定的 Hessian 矩阵，因此即使在 Hessian 矩阵不是正定的情况下也能正常工作，更适合这种情况。
    */

    // let mut opt: GaussNewton = GaussNewton::new(graph.clone());
    let mut opt: LevenMarquardt = LevenMarquardt::new(graph.clone());
    
    let result = opt.optimize(values).expect("优化失败");

    println!("saving optimization results ...");

    save_g2o_file("ch10-pose_graph_g2o_SE3-result.g2o", &result, &graph).expect("保存g2o文件失败");

    Ok(())
}

fn save_g2o_file(file_path: &str, values: &Values, graph: &Graph) -> io::Result<()> {
    let mut file = File::create(file_path)?;

    // TODO: 保存顶点和保存边
    

    // FIXME: 这里直接保存整个graph没有格式化了
    writeln!(file, "{:#?}", graph)?;

    Ok(())
}