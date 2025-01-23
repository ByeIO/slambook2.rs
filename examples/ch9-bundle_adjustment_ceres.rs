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

// 使用gomez库中的优化相关模块
use gomez::{
    Domain, Function, Optimizer, OptimizerDriver, 
    Problem, Sample
}; 
// 使用nalgebra库中的存储和向量相关模块
use gomez::nalgebra::{Dyn, Vector, storage::StorageMut, IsContiguous, Storage};
use fastrand::Rng;

// 标准库
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

struct ReprojectionError {
    observed_x: f64,
    observed_y: f64,
}

impl ReprojectionError {
    fn new(observed_x: f64, observed_y: f64) -> Self {
        Self { observed_x, observed_y }
    }
}

impl Problem for ReprojectionError {
    type Field = f64;

    fn domain(&self) -> Domain<Self::Field> {
        Domain::rect(vec![-10.0; 6], vec![10.0; 6])
    }
}

impl Function for ReprojectionError {
    fn apply<Sx>(&self, params: &Vector<Self::Field, Dyn, Sx>) -> Self::Field
    where
        Sx: Storage<Self::Field, Dyn> + IsContiguous, // 要求存储类型是连续的
    {
        let camera_params = params.fixed_view::<3, 1>(0, 0);
        let point_params = params.fixed_view::<3, 1>(3, 0);

        let projected_x = camera_params[0] * point_params[0] + camera_params[1] * point_params[1] + camera_params[2];
        let projected_y = camera_params[1] * point_params[0] + camera_params[0] * point_params[1] + camera_params[2];

        let residual_x = self.observed_x - projected_x;
        let residual_y = self.observed_y - projected_y;

        residual_x * residual_x + residual_y * residual_y
    }
}

struct Random {
    rng: Rng,
}

impl Random {
    fn new(rng: Rng) -> Self {
        Self { rng }
    }
}

impl<F: Function> Optimizer<F> for Random
where
    F::Field: Sample,
{
    const NAME: &'static str = "Random";
    type Error = std::convert::Infallible;

    fn opt_next<Sx>(
        &mut self,
        f: &F,
        dom: &Domain<F::Field>,
        x: &mut Vector<F::Field, Dyn, Sx>,
    ) -> Result<F::Field, Self::Error>
    where
        Sx: StorageMut<F::Field, Dyn> + IsContiguous,
    {
        dom.sample(x, &mut self.rng);
        Ok(f.apply(x))
    }
}

fn solve_ba(bal_problem: &RefCell<BALProblem>) {
    let mut bal_problem = bal_problem.borrow_mut();
    let point_block_size = bal_problem.point_block_size();
    let camera_block_size = bal_problem.camera_block_size();
    
    // 克隆或复制数据，避免多次借用
    let mut points = bal_problem.mutable_points().to_vec(); // 克隆 points
    let mut cameras = bal_problem.mutable_cameras().to_vec(); // 克隆 cameras
    let observations = bal_problem.observations().to_vec(); // 克隆 observations
    let camera_indices = bal_problem.camera_index().to_vec(); // 克隆 camera_indices
    let point_indices = bal_problem.point_index().to_vec(); // 克隆 point_indices
    let num_cameras = bal_problem.num_cameras();
    let num_observations = bal_problem.num_observations();

    for i in 0..num_observations as usize {
        let cost_function = ReprojectionError::new(observations[2 * i], observations[2 * i + 1]);

        let camera_index = camera_indices[i] as u32;
        let point_index = point_indices[i] as u32 + num_cameras as u32;

        let mut optimizer = OptimizerDriver::builder(&cost_function)
            .with_algo(|_, _| Random::new(Rng::new()))
            .build();

        let camera_index_size = camera_index as usize;
        let point_index_size = point_index as usize;
        
        let camera_block_size = camera_block_size as usize;
        let point_block_size = point_block_size as usize;
            
        let camera_len = cameras.len();
        let point_len = points.len();

        // 使用min防止下标越界
        let mut params = Vector::<f64, Dyn, gomez::nalgebra::VecStorage<f64, Dyn, gomez::nalgebra::Const<1>>>::from_vec(vec![
            cameras[(camera_index_size * camera_block_size).min(camera_len - 1)],
            cameras[(camera_index_size * camera_block_size + 1).min(camera_len - 1)],
            cameras[(camera_index_size * camera_block_size + 2).min(camera_len - 1)],
            points[(point_index_size * point_block_size).min(point_len - 1)],
            points[(point_index_size * point_block_size + 1).min(point_len - 1)],
            points[(point_index_size * point_block_size + 2).min(point_len - 1)],
        ]);

        optimizer
            .find(|state| {
                println!("f(x) = {}\tx = {:?}", state.fx(), state.x());
                state.iter() >= 100
            })
            .unwrap();

        // 更新 cameras 和 points,使用min防止下标越界
        cameras[(camera_index_size * camera_block_size).min(camera_len - 1)] = params[0];
        cameras[(camera_index_size * camera_block_size + 1).min(camera_len - 1)] = params[1];
        cameras[(camera_index_size * camera_block_size + 2).min(camera_len - 1)] = params[2];
        points[(point_index_size * point_block_size).min(point_len - 1)] = params[3];
        points[(point_index_size * point_block_size + 1).min(point_len - 1)] = params[4];
        points[(point_index_size * point_block_size + 2).min(point_len - 1)] = params[5];
    }

    // 将更新后的数据写回 bal_problem
    bal_problem.set_cameras(&cameras);
    bal_problem.set_points(&points);

    println!("bal problem file loaded...");
    println!("bal problem have {} cameras and {} points. ", num_cameras, bal_problem.num_points());
    println!("Forming {} observations. ", num_observations);

    println!("Solving BA using Gomez Random Optimizer ... ");
}

fn main() {
    let bal_problem = RefCell::new(BALProblem::new("./assets/ch9-problem-16-22106-pre.txt", false));
    bal_problem.borrow_mut().normalize();
    bal_problem.borrow_mut().perturb(0.1, 0.5, 0.5);
    bal_problem.borrow().write_to_ply_file("ch9-bundle_adjustment_ceres-initial.ply").expect("写入初始 PLY 文件失败");
    solve_ba(&bal_problem);
    bal_problem.borrow().write_to_ply_file("ch9-bundle_adjustment_ceres-final.ply").expect("写入最终 PLY 文件失败");
}