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
use nalgebra::{
    DMatrix, DVector, Matrix3, Vector3, 
    Vector2, Matrix6, Vector6, Matrix2x6, 
    SMatrix, SVector
};

// 图像处理
use image::{
    open, ImageBuffer, Rgb, DynamicImage, Luma
};
use imageproc::{
    drawing::draw_cross_mut, drawing::draw_line_segment_mut
};

// ORB角点检测
use bye_orb_rs::{
    orb, fast, common::Matchable
};

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
    noise::{GaussianNoise}
};

// 随机数
use rand::{
    Rng, SeedableRng
};
use rand::distributions::{
    Distribution, Uniform
};

// 时间
use std::time::Instant;

// 定义符号变量
assign_symbols!(X: VectorVar3);
assign_symbols!(Y: SE2);
assign_symbols!(Z: SE3);

fn main() {
    let img1_path = "./assets/ch7-1.png";
    let img2_path = "./assets/ch7-2.png";
    let img1_depth_path = "./assets/ch7-1_depth.png";
    let img2_depth_path = "./assets/ch7-2_depth.png";

    // 进行PnP估计(基于特征点的3D-2D姿态估计)
    main_pnp(img1_path, img2_path, img1_depth_path, img2_depth_path);
}

fn main_pnp(img1_path: &str, img2_path: &str, img1_depth_path: &str, img2_depth_path: &str) {
    // 读取图像
    let mut img1 = open(img1_path).unwrap();
    let mut img2 = open(img2_path).unwrap();

    // 读取深度图
    let depth1 = open(img1_depth_path).unwrap().to_luma16();
    let depth2 = open(img2_depth_path).unwrap().to_luma16();

    // 设置关键点数量
    let n_keypoints = 500;

    // 第一步: 检测Oriented FAST角点位置并计算BRIEF描述子
    let start_time = Instant::now();
    let img1_keypoints = orb::orb(&mut img1, n_keypoints).unwrap();
    let img2_keypoints = orb::orb(&mut img2, n_keypoints).unwrap();
    let end_time = Instant::now();
    println!("提取ORB特征点耗时: {:?} 秒", end_time - start_time);

    // 第二步: 使用Hamming距离进行匹配
    let start_time = Instant::now();
    let pair_indices = orb::match_brief(&img1_keypoints, &img2_keypoints);
    let end_time = Instant::now();
    println!("匹配ORB特征点耗时: {:?} 秒", end_time - start_time);

    // 第三步: 匹配点对筛选
    let mut matches: Vec<(usize, usize, f32)> = pair_indices.iter()
        .map(|&(i, j)| (i, j, img1_keypoints[i].distance(&img2_keypoints[j]) as f32))
        .collect();

    matches.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

    let min_dist = matches[0].2;
    let max_dist = matches[matches.len() - 1].2;

    println!("-- 最大距离: {}", max_dist);
    println!("-- 最小距离: {}", min_dist);

    // 当描述子之间的距离大于两倍的最小距离时，认为匹配有误。但最小距离可能会非常小，设置一个经验值30作为下限。
    let good_matches: Vec<(usize, usize, f32)> = matches.iter()
        .filter(|&&(_, _, dist)| dist <= (2.0 * min_dist).max(30.0))
        .cloned()
        .collect();

    // 第四步: 创建3D点和2D点
    let K = Matrix3::new(
        520.9, 0.0, 325.1,
        0.0, 521.0, 249.7,
        0.0, 0.0, 1.0
    );

    let mut pts_3d = Vec::new();
    let mut pts_2d = Vec::new();

    for &(i, j, _) in &good_matches {
        let d = depth1.get_pixel(img1_keypoints[i].x as u32, img1_keypoints[i].y as u32)[0] as f64;
        if d == 0.0 {
            continue;
        }
        let dd = d / 5000.0;
        let p1 = pixel2cam(Vector2::new(img1_keypoints[i].x as f64, img1_keypoints[i].y as f64), &K);
        pts_3d.push(Vector3::new(p1.x * dd, p1.y * dd, dd));
        pts_2d.push(Vector2::new(img2_keypoints[j].x as f64, img2_keypoints[j].y as f64));
    }

    println!("3d-2d pairs: {}", pts_3d.len());

    // 第五步: 使用Gauss-Newton进行Bundle Adjustment
    let mut pose_gn = SE3::identity();
    bundle_adjustment_gauss_newton(&pts_3d, &pts_2d, &K, &mut pose_gn);

    // 第六步: 使用G2O进行Bundle Adjustment
    let mut pose_g2o = SE3::identity();
    bundle_adjustment_g2o(&pts_3d, &pts_2d, &K, &mut pose_g2o);
}

fn pixel2cam(p: Vector2<f64>, K: &Matrix3<f64>) -> Vector2<f64> {
    Vector2::new(
        (p.x - K[(0, 2)]) / K[(0, 0)],
        (p.y - K[(1, 2)]) / K[(1, 1)]
    )
}

fn bundle_adjustment_gauss_newton(
    points_3d: &Vec<Vector3<f64>>,
    points_2d: &Vec<Vector2<f64>>,
    K: &Matrix3<f64>,
    pose: &mut SE3<f64>,
) {
    let iterations = 10;
    let mut cost = 0.0;
    let mut last_cost = 0.0;
    let fx = K[(0, 0)];
    let fy = K[(1, 1)];
    let cx = K[(0, 2)];
    let cy = K[(1, 2)];

    for iter in 0..iterations {
        let mut H = Matrix6::zeros();
        let mut b = Vector6::zeros();

        cost = 0.0;

        for i in 0..points_3d.len() {

            // 假设 pose 是 SE3 类型的对象，points_3d 是 Vector3 类型的数组
            let pc = pose.apply(points_3d[i].as_view());

            let inv_z = 1.0 / pc[2];
            let inv_z2 = inv_z * inv_z;
            let proj = Vector2::new(fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);

            let e = points_2d[i] - proj;

            cost += e.norm_squared();

            let mut J = Matrix2x6::zeros();
            J[(0, 0)] = -fx * inv_z;
            J[(0, 2)] = fx * pc[0] * inv_z2;
            J[(0, 3)] = fx * pc[0] * pc[1] * inv_z2;
            J[(0, 4)] = -fx - fx * pc[0] * pc[0] * inv_z2;
            J[(0, 5)] = fx * pc[1] * inv_z;
            J[(1, 1)] = -fy * inv_z;
            J[(1, 2)] = fy * pc[1] * inv_z2;
            J[(1, 3)] = fy + fy * pc[1] * pc[1] * inv_z2;
            J[(1, 4)] = -fy * pc[0] * pc[1] * inv_z2;
            J[(1, 5)] = -fy * pc[0] * inv_z;

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        let dx = H.lu().solve(&b).unwrap();

        if dx[0].is_nan() {
            println!("result is nan!");
            break;
        }

        if iter > 0 && cost >= last_cost {
            println!("cost: {}, last cost: {}", cost, last_cost);
            break;
        }

        *pose = SE3::exp((&dx).into()) * pose.clone();
        last_cost = cost;

        println!("iteration {} cost={}", iter, cost);
        if dx.norm() < 1e-6 {
            break;
        }
    }

    println!("pose by g-n: \n{}", pose.to_matrix());
}

fn bundle_adjustment_g2o(
    points_3d: &Vec<Vector3<f64>>,
    points_2d: &Vec<Vector2<f64>>,
    K: &Matrix3<f64>,
    pose: &mut SE3<f64>,
) {
    let mut graph = Graph::new();

    let mut values = Values::new();
    let angle = pose.rot().log().norm(); // 计算旋转的角度
    values.insert(X(0), VectorVar3::new(pose.xyz().x, pose.xyz().y, angle));

    for i in 0..points_2d.len() {
        let factor = CurveFittingFactor::new(points_3d[i].x, points_2d[i].x);
        let noise_model = GaussianNoise::<1>::identity();
        let factor_node = fac![factor, X(0), noise_model];
        graph.add_factor(factor_node);
    }

    let mut opt: GaussNewton = GaussNewton::new(graph);
    let result = opt.optimize(values).expect("优化失败");

    println!("最终结果: {:#?}", result);
}

// 曲线拟合的因子
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

// 实现 Residual1 trait 用于因子
// `mark` 宏处理序列化内容以及一些自定义实现
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
        // 返回值
        VectorX::from_column_slice(&[error])
    }

}