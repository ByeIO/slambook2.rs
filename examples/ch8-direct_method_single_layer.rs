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
    open, ImageBuffer, Rgb, DynamicImage, Luma, RgbImage,
    buffer::ConvertBuffer
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

// 引用单元
use std::cell::RefCell;

// 定义符号变量
assign_symbols!(_X_: VectorVar3);
assign_symbols!(_Y_: SE2);
assign_symbols!(_Z_: SE3);

// 相机内参
const FX: f64 = 718.856;
const FY: f64 = 718.856;
const CX: f64 = 607.1928;
const CY: f64 = 185.2157;
// 基线
const BASELINE: f64 = 0.573;

// 双线性插值
fn get_pixel_value(img: &ImageBuffer<Luma<u8>, Vec<u8>>, x: f64, y: f64) -> f64 {
    let mut x = x;
    let mut y = y;
    
    // 边界检查
    if x < 0.0 {
        x = 0.0;
    }
    if y < 0.0 {
        y = 0.0;
    }
    if x >= img.width() as f64 - 1.0 {
        x = img.width() as f64 - 1.0;
    }
    if y >= img.height() as f64 - 1.0 {
        y = img.height() as f64 - 1.0;
    }

    let x_floor = x.floor() as u32;
    let y_floor = y.floor() as u32;
    let xx = x - x_floor as f64;
    let yy = y - y_floor as f64;

    // 确保不越界
    let x_ceil = (x_floor + 1).min(img.width() - 1);
    let y_ceil = (y_floor + 1).min(img.height() - 1);

    let data = |x: u32, y: u32| img.get_pixel(x, y)[0] as f64;

    (1.0 - xx) * (1.0 - yy) * data(x_floor, y_floor) +
    xx * (1.0 - yy) * data(x_ceil, y_floor) +
    (1.0 - xx) * yy * data(x_floor, y_ceil) +
    xx * yy * data(x_ceil, y_ceil)
}

// 用于并行累加雅可比矩阵的结构体
struct JacobianAccumulator<'a> {
    img1: &'a ImageBuffer<Luma<u8>, Vec<u8>>,
    img2: &'a ImageBuffer<Luma<u8>, Vec<u8>>,
    px_ref: &'a Vec<Vector2<f64>>,
    depth_ref: &'a Vec<f64>,
    T21: &'a mut SE3<f64>,
    projection: Vec<Vector2<f64>>,
    H: Matrix6<f64>,
    b: Vector6<f64>,
    cost: f64,
}

impl<'a> JacobianAccumulator<'a> {
    fn new(
        img1: &'a ImageBuffer<Luma<u8>, Vec<u8>>,
        img2: &'a ImageBuffer<Luma<u8>, Vec<u8>>,
        px_ref: &'a Vec<Vector2<f64>>,
        depth_ref: &'a Vec<f64>,
        T21: &'a mut SE3<f64>
    ) -> Self {
        let projection = vec![Vector2::zeros(); px_ref.len()];
        Self {
            img1,
            img2,
            px_ref,
            depth_ref,
            T21,
            projection,
            H: Matrix6::zeros(),
            b: Vector6::zeros(),
            cost: 0.0,
        }
    }

    fn accumulate_jacobian(&mut self, range: std::ops::Range<usize>) {
        let half_patch_size = 1;  
        let mut cnt_good = 0;
        let mut hessian = Matrix6::zeros();
        let mut bias = Vector6::zeros();
        let mut cost_tmp = 0.0;

        for i in range {
            // 计算在第二张图像中的投影
            let point_ref = self.depth_ref[i] * Vector3::new(
                (self.px_ref[i].x - CX) / FX,
                (self.px_ref[i].y - CY) / FY,
                1.0
            );
            let point_cur = self.T21.apply(point_ref.as_view());
            if point_cur[2] < 0.0 {   // 深度无效
                continue;
            }

            let u = FX * point_cur[0] / point_cur[2] + CX;
            let v = FY * point_cur[1] / point_cur[2] + CY;
            if u < half_patch_size as f64 || u > (self.img2.width() - half_patch_size) as f64 ||
                v < half_patch_size as f64 || v > (self.img2.height() - half_patch_size) as f64 {
                continue;
            }

            self.projection[i] = Vector2::new(u, v);
            let X = point_cur[0];
            let Y = point_cur[1];
            let Z = point_cur[2];
            let Z2 = Z * Z;
            let Z_inv = 1.0 / Z;
            let Z2_inv = Z_inv * Z_inv;
            cnt_good += 1;

            // 计算误差和雅可比
            for x in -(half_patch_size as i32)..=half_patch_size as i32 {
                for y in -(half_patch_size as i32)..=half_patch_size as i32 {
                    let error = get_pixel_value(self.img1, self.px_ref[i].x + x as f64, self.px_ref[i].y + y as f64) -
                                get_pixel_value(self.img2, u + x as f64, v + y as f64);
                    let mut J_pixel_xi = Matrix2x6::zeros();
                    let mut J_img_pixel = Vector2::zeros();

                    J_pixel_xi[(0, 0)] = FX * Z_inv;
                    J_pixel_xi[(0, 1)] = 0.0;
                    J_pixel_xi[(0, 2)] = -FX * X * Z2_inv;
                    J_pixel_xi[(0, 3)] = -FX * X * Y * Z2_inv;
                    J_pixel_xi[(0, 4)] = FX + FX * X * X * Z2_inv;
                    J_pixel_xi[(0, 5)] = -FX * Y * Z_inv;

                    J_pixel_xi[(1, 0)] = 0.0;
                    J_pixel_xi[(1, 1)] = FY * Z_inv;
                    J_pixel_xi[(1, 2)] = -FY * Y * Z2_inv;
                    J_pixel_xi[(1, 3)] = -FY - FY * Y * Y * Z2_inv;
                    J_pixel_xi[(1, 4)] = FY * X * Y * Z2_inv;
                    J_pixel_xi[(1, 5)] = FY * X * Z_inv;

                    J_img_pixel = Vector2::new(
                        0.5 * (get_pixel_value(self.img2, u + 1.0 + x as f64, v + y as f64) - get_pixel_value(self.img2, u - 1.0 + x as f64, v + y as f64)),
                        0.5 * (get_pixel_value(self.img2, u + x as f64, v + 1.0 + y as f64) - get_pixel_value(self.img2, u + x as f64, v - 1.0 + y as f64))
                    );

                    // 总雅可比
                    let J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();

                    hessian += J * J.transpose();
                    bias += -error * J;
                    cost_tmp += error * error;
                }
            }
        }

        if cnt_good > 0 {
            self.H += hessian;
            self.b += bias;
            self.cost += cost_tmp / cnt_good as f64;
        }
    }

    fn hessian(&self) -> Matrix6<f64> {
        self.H
    }

    fn bias(&self) -> Vector6<f64> {
        self.b
    }

    fn cost_func(&self) -> f64 {
        self.cost
    }

    fn projected_points(&self) -> &Vec<Vector2<f64>> {
        &self.projection
    }

    fn reset(&mut self) {
        self.H = Matrix6::zeros();
        self.b = Vector6::zeros();
        self.cost = 0.0;
    }
}

fn direct_pose_estimation_single_layer(
    img1: &ImageBuffer<Luma<u8>, Vec<u8>>,
    img2: &ImageBuffer<Luma<u8>, Vec<u8>>,
    px_ref: &Vec<Vector2<f64>>,
    depth_ref: &Vec<f64>,
    T21: &mut SE3<f64>
) {
    let iterations = 10;
    let mut cost = 0.0;
    let mut last_cost = 0.0;
    let t1 = Instant::now();

    for iter in 0..iterations {
        // 使用 T21 的克隆来避免借用冲突
        let mut T21_clone = T21.clone();
        let mut jaco_accu = JacobianAccumulator::new(img1, img2, px_ref, depth_ref, &mut T21_clone);
        jaco_accu.reset();
        jaco_accu.accumulate_jacobian(0..px_ref.len());
        let H = jaco_accu.hessian();
        let b = jaco_accu.bias();

        // 求解更新并放入估计
        let update = H.lu().solve(&b).unwrap_or_else(|| {
            eprintln!("矩阵求解失败!");
            Vector6::zeros()
        });
        if update[0].is_nan() {
            // 有时发生在我们有一个黑色或白色的补丁并且H是不可逆的
            eprintln!("更新为nan");
            break;
        }

        // 更新 T21
        *T21 = SE3::exp((&update).into()) * T21.clone();
        cost = jaco_accu.cost_func();

        if iter > 0 && cost > last_cost {
            eprintln!("成本增加: {}, {}", cost, last_cost);
            break;
        }
        if update.norm() < 1e-3 {
            // 收敛
            break;
        }

        last_cost = cost;
        println!("迭代: {}, 成本: {}", iter, cost);
    }

    println!("T21 = \n{}", T21.to_matrix());
    let t2 = Instant::now();
    let time_used = t2.duration_since(t1).as_secs_f64();
    println!("单层直接法: {}", time_used);

    // 在此处绘制投影像素,转换为rgb图
    let mut img2_show: RgbImage = img2.convert();

    let mut T21_clone = T21.clone();
    let mut jaco_accu = JacobianAccumulator::new(img1, img2, px_ref, depth_ref, &mut T21_clone);
    let projection = jaco_accu.projected_points();
    for i in 0..px_ref.len() {
        let p_ref = px_ref[i];
        let p_cur = projection[i];
        if p_cur.x > 0.0 && p_cur.y > 0.0 {
            draw_cross_mut(&mut img2_show, Rgb([0, 250, 0]), p_cur.x as i32, p_cur.y as i32);
            draw_line_segment_mut(&mut img2_show, (p_ref.x as f32, p_ref.y as f32), (p_cur.x as f32, p_cur.y as f32), Rgb([0, 250, 0]));
        }
    }
    img2_show.save("ch8-direct_method-current.png").unwrap();
}

// 多层直接法位姿估计
fn direct_pose_estimation_multi_layer(
    img1: &ImageBuffer<Luma<u8>, Vec<u8>>,
    img2: &ImageBuffer<Luma<u8>, Vec<u8>>,
    px_ref: &Vec<Vector2<f64>>,
    depth_ref: &Vec<f64>,
    T21: &mut SE3<f64>
) {
    // 参数
    let pyramids = 4;
    let pyramid_scale = 0.5;
    let scales = [1.0, 0.5, 0.25, 0.125];

    // 创建金字塔
    let mut pyr1 = Vec::new();
    let mut pyr2 = Vec::new();
    for i in 0..pyramids {
        if i == 0 {
            pyr1.push(img1.clone());
            pyr2.push(img2.clone());
        } else {
            let img1_pyr = image::imageops::resize(&pyr1[i - 1], (pyr1[i - 1].width() as f64 * pyramid_scale) as u32, (pyr1[i - 1].height() as f64 * pyramid_scale) as u32, image::imageops::FilterType::Nearest);
            let img2_pyr = image::imageops::resize(&pyr2[i - 1], (pyr2[i - 1].width() as f64 * pyramid_scale) as u32, (pyr2[i - 1].height() as f64 * pyramid_scale) as u32, image::imageops::FilterType::Nearest);

            // 转换图片为灰度图,直接将luma8数据推入向量
            pyr1.push(img1_pyr);
            pyr2.push(img2_pyr);
        }
    }

    let fxG = FX;
    let fyG = FY;
    let cxG = CX;
    let cyG = CY;  // 备份旧值
    for level in (0..pyramids).rev() {
        let mut px_ref_pyr = Vec::new();
        for px in px_ref {
            px_ref_pyr.push(scales[level] * px);
        }

        // 在不同的金字塔层次中缩放 fx, fy, cx, cy
        let fx = fxG * scales[level];
        let fy = fyG * scales[level];
        let cx = cxG * scales[level];
        let cy = cyG * scales[level];
        direct_pose_estimation_single_layer(&pyr1[level], &pyr2[level], &px_ref_pyr, depth_ref, T21);
    }
}

fn main() {
    let left_file = "./assets/ch8-left.png";
    let disparity_file = "./assets/ch8-disparity.png";
    let fmt_others = "./assets/ch8-{:06}.png"; // 注意这里是 {:06}，不是 %06d

    let left_img = open(left_file).unwrap().to_luma8();
    let disparity_img = open(disparity_file).unwrap().to_luma8();

    // 我们在第一张图像中随机选择像素并在第一张图像的帧中生成一些3D点
    let mut rng = rand::thread_rng();
    let nPoints = 2000;
    let boarder = 20;
    let mut pixels_ref = Vec::new();
    let mut depth_ref = Vec::new();

    // 在ref中生成像素并加载深度数据
    for _ in 0..nPoints {
        let x = rng.gen_range(boarder..left_img.width() - boarder) as f64;
        let y = rng.gen_range(boarder..left_img.height() - boarder) as f64;
        let disparity = disparity_img.get_pixel(x as u32, y as u32)[0] as f64;
        let depth = FX * BASELINE / disparity; // 你知道这是视差到深度
        depth_ref.push(depth);
        pixels_ref.push(Vector2::new(x, y));
    } // 结束for

    // 使用此信息估计01~05.png的姿态
    let mut T_cur_ref = SE3::identity();

    for i in 1..6 {
        let img_path = format!("./assets/ch8-{:06}.png", i);
        println!("图片路径:{:?}\n",img_path);
        let img = open(&img_path).unwrap().to_luma8();
        // 通过取消注释此行尝试单层
        direct_pose_estimation_single_layer(&left_img, &img, &pixels_ref, &depth_ref, &mut T_cur_ref);
    }

}