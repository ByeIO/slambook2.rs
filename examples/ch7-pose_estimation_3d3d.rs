#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(unused_assignments)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
#![allow(rustdoc::missing_crate_level_docs)]
#![allow(unsafe_code)]
#![allow(clippy::undocumented_unsafe_blocks)]
#![allow(unused_must_use)]
#![allow(non_snake_case)]

// 线性代数
use nalgebra::{DMatrix, DVector, Matrix3, Vector3, Vector2, Matrix6, Vector6, Matrix2x6, SMatrix, SVector};
// 图像处理
use image::{open, ImageBuffer, Rgb, DynamicImage, Luma};
use imageproc::{drawing::draw_cross_mut, drawing::draw_line_segment_mut};
// ORB角点检测
use bye_orb_rs::{orb, fast, common::Matchable};
// 图优化&李代数
use factrs::{
    assign_symbols,
    core::{BetweenResidual, GaussNewton, Graph, Values},
    dtype, fac,
    linalg::{Const, ForwardProp, Numeric, NumericalDiff, VectorX, DiffResult, MatrixX},
    residuals::Residual1,
    traits::*,
    variables::{VectorVar2, SE2, VectorVar3, SE3, SO2, SO3, MatrixLieGroup},
    containers::Key,
    noise::{GaussianNoise}
};
// 随机数
use rand::{Rng, SeedableRng};
use rand::distributions::{Distribution, Uniform};
// 时间
use std::time::Instant;

assign_symbols!(X: VectorVar3);
assign_symbols!(Y: SE2);
assign_symbols!(Z: SE3);

fn main() {
    let img1_path = "./assets/ch7-1.png";
    let img2_path = "./assets/ch7-2.png";
    let img1_depth_path = "./assets/ch7-1_depth.png";
    let img2_depth_path = "./assets/ch7-2_depth.png";

    main_3d3d(img1_path, img2_path, img1_depth_path, img2_depth_path);
}

fn main_3d3d(img1_path: &str, img2_path: &str, img1_depth_path: &str, img2_depth_path: &str) {
    let mut img1 = open(img1_path).unwrap();
    let mut img2 = open(img2_path).unwrap();

    let depth1 = open(img1_depth_path).unwrap().to_luma16();
    let depth2 = open(img2_depth_path).unwrap().to_luma16();

    let n_keypoints = 500;

    let start_time = Instant::now();
    let img1_keypoints = orb::orb(&mut img1, n_keypoints).unwrap();
    let img2_keypoints = orb::orb(&mut img2, n_keypoints).unwrap();
    let end_time = Instant::now();
    println!("提取ORB特征点耗时: {:?} 秒", end_time - start_time);

    let start_time = Instant::now();
    let pair_indices = orb::match_brief(&img1_keypoints, &img2_keypoints);
    let end_time = Instant::now();
    println!("匹配ORB特征点耗时: {:?} 秒", end_time - start_time);

    let mut matches: Vec<(usize, usize, f32)> = pair_indices.iter()
        .map(|&(i, j)| (i, j, img1_keypoints[i].distance(&img2_keypoints[j]) as f32))
        .collect();

    matches.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

    let min_dist = matches[0].2;
    let max_dist = matches[matches.len() - 1].2;

    println!("-- 最大距离: {}", max_dist);
    println!("-- 最小距离: {}", min_dist);

    let good_matches: Vec<(usize, usize, f32)> = matches.iter()
        .filter(|&&(_, _, dist)| dist <= (2.0 * min_dist).max(30.0))
        .cloned()
        .collect();

    let K = Matrix3::new(
        520.9, 0.0, 325.1,
        0.0, 521.0, 249.7,
        0.0, 0.0, 1.0
    );

    let mut pts1 = Vec::new();
    let mut pts2 = Vec::new();

    for &(i, j, _) in &good_matches {
        let d1 = depth1.get_pixel(img1_keypoints[i].x as u32, img1_keypoints[i].y as u32)[0] as f64;
        let d2 = depth2.get_pixel(img2_keypoints[j].x as u32, img2_keypoints[j].y as u32)[0] as f64;
        if d1 == 0.0 || d2 == 0.0 {
            continue;
        }
        let dd1 = d1 / 5000.0;
        let dd2 = d2 / 5000.0;
        let p1 = pixel2cam(Vector2::new(img1_keypoints[i].x as f64, img1_keypoints[i].y as f64), &K);
        let p2 = pixel2cam(Vector2::new(img2_keypoints[j].x as f64, img2_keypoints[j].y as f64), &K);
        pts1.push(Vector3::new(p1.x * dd1, p1.y * dd1, dd1));
        pts2.push(Vector3::new(p2.x * dd2, p2.y * dd2, dd2));
    }

    println!("3d-3d pairs: {}", pts1.len());

    let mut pose = SE3::identity();
    pose_estimation_3d3d(&pts1, &pts2, &mut pose);
    println!("ICP via SVD results: \n{}", pose.to_matrix());

    bundle_adjustment_g2o(&pts1, &pts2, &mut pose);
    println!("After optimization: \n{}", pose.to_matrix());
}

fn pixel2cam(p: Vector2<f64>, K: &Matrix3<f64>) -> Vector2<f64> {
    Vector2::new(
        (p.x - K[(0, 2)]) / K[(0, 0)],
        (p.y - K[(1, 2)]) / K[(1, 1)]
    )
}

fn pose_estimation_3d3d(pts1: &Vec<Vector3<f64>>, pts2: &Vec<Vector3<f64>>, pose: &mut SE3<f64>) {
    let mut p1 = Vector3::zeros();
    let mut p2 = Vector3::zeros();
    let n = pts1.len();

    for i in 0..n {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 /= n as f64;
    p2 /= n as f64;

    let mut q1 = Vec::new();
    let mut q2 = Vec::new();
    for i in 0..n {
        q1.push(pts1[i] - p1);
        q2.push(pts2[i] - p2);
    }

    let mut W = Matrix3::zeros();
    for i in 0..n {
        W += q1[i] * q2[i].transpose();
    }

    let svd = W.svd(true, true);
    let U = svd.u.unwrap();
    let V = svd.v_t.unwrap().transpose();

    let mut R = U * V.transpose();
    if R.determinant() < 0.0 {
        R = -R;
    }

    let t = p1 - R * p2;

    let so3_rotation = SO3::from_matrix(R.fixed_view::<3, 3>(0, 0).into());
    *pose = SE3::from_rot_trans(so3_rotation, t.into());

}

fn bundle_adjustment_g2o(pts1: &Vec<Vector3<f64>>, pts2: &Vec<Vector3<f64>>, pose: &mut SE3<f64>) {
    let mut graph = Graph::new();

    let mut values = Values::new();
    let angle = pose.rot().log().norm();
    values.insert(X(0), VectorVar3::new(pose.xyz().x, pose.xyz().y, angle));

    for i in 0..pts2.len() {
        let factor = CurveFittingFactor::new(pts2[i].x, pts1[i].x);
        let noise_model = GaussianNoise::<1>::identity();
        let factor_node = fac![factor, X(0), noise_model];
        graph.add_factor(factor_node);
    }

    let mut opt: GaussNewton = GaussNewton::new(graph);
    let result = opt.optimize(values).expect("优化失败");

    println!("最终结果: {:#?}", result);
}

#[derive(Clone, Debug)]
pub struct CurveFittingFactor {
    x: f64,
    measurement: f64,
}

impl CurveFittingFactor {
    pub fn new(x: f64, measurement: f64) -> Self {
        Self { x, measurement }
    }
}

#[factrs::mark]
impl Residual1 for CurveFittingFactor {
    type Differ = ForwardProp<<Self as Residual1>::DimIn>;
    type V1 = VectorVar3;
    type DimIn = Const<3>;
    type DimOut = Const<1>;

    fn residual1<T: Numeric>(&self, v: VectorVar3<T>) -> VectorX<T> {
        let abc = v.to_owned();
        let x_squared = T::from_f64(self.x * self.x).unwrap_or_else(|| T::from_f64(0.0).unwrap());
        let x = T::from_f64(self.x).unwrap_or_else(|| T::from_f64(0.0).unwrap());
        let measurement = T::from_f64(self.measurement).unwrap_or_else(|| T::from_f64(0.0).unwrap());
        println!("abc: {:#?}", abc);
        let error = measurement - (abc[0] * x_squared + abc[1] * x + abc[2]).exp();
        VectorX::from_column_slice(&[error])
    }
}
