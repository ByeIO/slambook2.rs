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

// 线性代数
use nalgebra::{DMatrix, DVector, Matrix3, Vector3, SVector, Matrix, VecStorage, Dyn};

// 图优化
use factrs::{
    assign_symbols,
    core::{BetweenResidual, GaussNewton, Graph, Values},
    dtype, fac,
    linalg::{Const, ForwardProp, Numeric, NumericalDiff, VectorX, DiffResult, MatrixX},
    residuals::Residual1,
    traits::*,
    variables::{VectorVar2, SE2, VectorVar3, SE3, SO2, SO3, MatrixLieGroup},
    containers::Key,
    noise::{GaussianNoise},
    robust::{Huber, Cauchy},
    optimizers::Optimizer
};

use rand::{Rng, SeedableRng};
use rand::distributions::{Distribution, Uniform};

use std::process::exit;
use std::ops::Add;
use std::ops::Mul;
use std::io;
use std::cell::{
    RefMut, Ref, RefCell
};

// 使用本地库
#[path = "../src/common.rs"]
mod common;
#[path = "../src/bundle_adjustment.rs"]
mod bundle_adjustment;
use common::{get_pixel_value, rand_double, rand_normal, dot_product, cross_product, angle_axis_to_quaternion, quaternion_to_angle_axis, angle_axis_rotate_point, sqrt, sin, cos, atan2};
use bundle_adjustment::{SnavelyReprojectionError, BALProblem};

// 定义符号变量
assign_symbols!(CAMERA: VectorVar3);
assign_symbols!(POINT: VectorVar3);

// ba问题求解
fn solve_ba(mut bal_problem: BALProblem) {
    let point_block_size = bal_problem.point_block_size();
    let camera_block_size = bal_problem.camera_block_size();
    
    // 克隆或复制数据，避免多次借用
    let observations = bal_problem.observations().to_vec();  // 克隆 observations
    let mut points = bal_problem.mutable_points().to_vec();  // 克隆 points
    let mut cameras = bal_problem.mutable_cameras().to_vec();  // 克隆 cameras
    let camera_indices = bal_problem.camera_index().to_vec();  // 克隆 camera_indices
    let point_indices = bal_problem.point_index().to_vec();  // 克隆 point_indices
    let num_cameras = bal_problem.num_cameras();
    let num_points = bal_problem.num_points();
    let num_observations = bal_problem.num_observations();

    let mut graph = Graph::new();

    for i in 0..observations.len() / 2 {
        let cost_function = SnavelyReprojectionError::new(observations[2 * i], observations[2 * i + 1]);
        let inv_sigma = 1.0;

        let camera_index = camera_indices[i] as u32;
        let point_index = point_indices[i] as u32 + num_cameras as u32;

        let cost_function_clone = cost_function.clone();  // 克隆 cost_function

        let factor_node1 = fac![cost_function, CAMERA(camera_index), GaussianNoise::<2>::identity(), Huber::new(inv_sigma)];
        graph.add_factor(factor_node1);

        let factor_node2 = fac![cost_function_clone, POINT(point_index), GaussianNoise::<2>::identity(), Huber::new(inv_sigma)];
        graph.add_factor(factor_node2);
    }

    println!("bal problem file loaded...");
    println!("bal problem have {} cameras and {} points. ", num_cameras, num_points);
    println!("Forming {} observations. ", num_observations);

    println!("Solving factrs BA ... ");
    let mut values = Values::new();
    for i in 0..num_cameras as usize {
        let start = i * camera_block_size as usize;
        let end = start + camera_block_size as usize;
        let camera_slice = &cameras[start..end];
        values.insert(CAMERA(i as u32), VectorVar3::new(camera_slice[0], camera_slice[1], camera_slice[2]));
    }
    for i in 0..num_points as usize {
        let start = i * point_block_size as usize;
        let end = start + point_block_size as usize;
        let point_slice = &points[start..end];
        values.insert(POINT((i + num_cameras as usize) as u32), VectorVar3::new(point_slice[0], point_slice[1], point_slice[2]));
    }

    let mut opt: GaussNewton = GaussNewton::new(graph);
    let result = opt.optimize(values).expect("优化失败");

    println!("{:#?}", result);

    // 将更新后的数据写回 bal_problem
    bal_problem.set_cameras(&cameras);
    bal_problem.set_points(&points);
}

// 主函数
fn main() {
    let mut bal_problem = BALProblem::new("./assets/ch9-problem-16-22106-pre.txt", false);
    bal_problem.normalize();
    bal_problem.perturb(0.1, 0.5, 0.5);
    bal_problem.write_to_ply_file("ch9-bundle_adjustment_g2o-initial.ply").expect("写入初始 PLY 文件失败");
    solve_ba(bal_problem.clone());
    bal_problem.write_to_ply_file("ch9-bundle_adjustment_g2o-final.ply").expect("写入最终 PLY 文件失败");
}