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
    SMatrix, SVector, Matrix2
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

// 光流跟踪器
struct OpticalFlowTracker<'a> {
    img1: &'a ImageBuffer<Luma<u8>, Vec<u8>>,
    img2: &'a ImageBuffer<Luma<u8>, Vec<u8>>,
    kp1: &'a Vec<Vector2<f64>>,
    kp2: &'a mut Vec<Vector2<f64>>,
    success: &'a mut Vec<bool>,
    inverse: bool,
    has_initial: bool,
}

impl<'a> OpticalFlowTracker<'a> {
    fn calculate_optical_flow(&mut self, range: std::ops::Range<usize>) {
        let half_patch_size = 4;
        let iterations = 10;
        for i in range {
            let kp = self.kp1[i];
            let mut dx = 0.0;
            let mut dy = 0.0;
            if self.has_initial {
                dx = self.kp2[i].x - kp.x;
                dy = self.kp2[i].y - kp.y;
            }

            let mut cost = 0.0;
            let mut last_cost = 0.0;
            let mut succ = true;

            let mut H = Matrix2::zeros();
            let mut b = Vector2::zeros();
            let mut J = Vector2::zeros();

            for iter in 0..iterations {
                if !self.inverse {
                    H = Matrix2::zeros();
                    b = Vector2::zeros();
                } else {
                    b = Vector2::zeros();
                }

                cost = 0.0;

                for x in -half_patch_size..half_patch_size {
                    for y in -half_patch_size..half_patch_size {
                        let error = get_pixel_value(self.img1, kp.x + x as f64, kp.y + y as f64) -
                                    get_pixel_value(self.img2, kp.x + x as f64 + dx, kp.y + y as f64 + dy);
                        if !self.inverse {
                            J = -Vector2::new(
                                0.5 * (get_pixel_value(self.img2, kp.x + dx + x as f64 + 1.0, kp.y + dy + y as f64) -
                                       get_pixel_value(self.img2, kp.x + dx + x as f64 - 1.0, kp.y + dy + y as f64)),
                                0.5 * (get_pixel_value(self.img2, kp.x + dx + x as f64, kp.y + dy + y as f64 + 1.0) -
                                       get_pixel_value(self.img2, kp.x + dx + x as f64, kp.y + dy + y as f64 - 1.0))
                            );
                        } else if iter == 0 {
                            J = -Vector2::new(
                                0.5 * (get_pixel_value(self.img1, kp.x + x as f64 + 1.0, kp.y + y as f64) -
                                       get_pixel_value(self.img1, kp.x + x as f64 - 1.0, kp.y + y as f64)),
                                0.5 * (get_pixel_value(self.img1, kp.x + x as f64, kp.y + y as f64 + 1.0) -
                                       get_pixel_value(self.img1, kp.x + x as f64, kp.y + y as f64 - 1.0))
                            );
                        }
                        b += -error * J;
                        cost += error * error;
                        if !self.inverse || iter == 0 {
                            H += J * J.transpose();
                        }
                    }
                }

                let update = H.lu().solve(&b).unwrap_or(Vector2::zeros());

                if update[0].is_nan() {
                    succ = false;
                    break;
                }

                if iter > 0 && cost > last_cost {
                    break;
                }

                dx += update[0];
                dy += update[1];
                last_cost = cost;
                succ = true;

                if update.norm() < 1e-2 {
                    break;
                }

            }

            self.success[i] = succ;
            self.kp2[i] = kp + Vector2::new(dx, dy);
        }// end for i
    }// end fn calculate_optical_flow
}

fn optical_flow_single_level(
    img1: &ImageBuffer<Luma<u8>, Vec<u8>>,
    img2: &ImageBuffer<Luma<u8>, Vec<u8>>,
    kp1: &Vec<Vector2<f64>>,
    kp2: &mut Vec<Vector2<f64>>,
    success: &mut Vec<bool>,
    inverse: bool,
    has_initial: bool,
) {
    // 初始化 kp2 和 success 向量
    kp2.resize(kp1.len(), Vector2::zeros());
    success.resize(kp1.len(), false);

    // 创建光流跟踪器
    let mut tracker = OpticalFlowTracker {
        img1,
        img2,
        kp1,
        kp2,
        success,
        inverse,
        has_initial,
    };

    // 计算光流
    tracker.calculate_optical_flow(0..kp1.len());
}

// 多层光流追踪
fn optical_flow_multi_level(
    img1: &ImageBuffer<Luma<u8>, Vec<u8>>,
    img2: &ImageBuffer<Luma<u8>, Vec<u8>>,
    kp1: &Vec<Vector2<f64>>,
    kp2: &mut Vec<Vector2<f64>>,
    success: &mut Vec<bool>,
    inverse: bool,
) {
    // 参数
    let pyramids = 4;
    let pyramid_scale = 0.5;
    let scales = [1.0, 0.5, 0.25, 0.125];

    // 创建图像金字塔
    let mut pyr1 = Vec::new();
    let mut pyr2 = Vec::new();
    for i in 0..pyramids {
        if i == 0 {
            pyr1.push(img1.clone());
            pyr2.push(img2.clone());
        } else {
            let img1_pyr = image::imageops::resize(
                &pyr1[i - 1],
                (pyr1[i - 1].width() as f64 * pyramid_scale) as u32,
                (pyr1[i - 1].height() as f64 * pyramid_scale) as u32,
                image::imageops::FilterType::Triangle,
            );
            let img2_pyr = image::imageops::resize(
                &pyr2[i - 1],
                (pyr2[i - 1].width() as f64 * pyramid_scale) as u32,
                (pyr2[i - 1].height() as f64 * pyramid_scale) as u32,
                image::imageops::FilterType::Triangle,
            );
            pyr1.push(img1_pyr);
            pyr2.push(img2_pyr);
        }
    }

    // 从粗到细的光流追踪
    let mut kp1_pyr = Vec::new();
    let mut kp2_pyr = Vec::new();
    for kp in kp1 {
        let kp_top = Vector2::new(kp.x * scales[pyramids - 1], kp.y * scales[pyramids - 1]);
        kp1_pyr.push(kp_top);
        kp2_pyr.push(kp_top);
    }

    for level in (0..pyramids).rev() {
        success.clear();
        optical_flow_single_level(
            &pyr1[level],
            &pyr2[level],
            &kp1_pyr,
            &mut kp2_pyr,
            success,
            inverse,
            true,
        );

        if level > 0 {
            for kp in &mut kp1_pyr {
                kp.x /= pyramid_scale;
                kp.y /= pyramid_scale;
            }
            for kp in &mut kp2_pyr {
                kp.x /= pyramid_scale;
                kp.y /= pyramid_scale;
            }
        }
    }

    // 将最终结果存入 kp2
    kp2.clear();
    for kp in kp2_pyr {
        kp2.push(kp);
    }
}

fn main() {
    // 读取图像
    let img1 = open("./assets/ch8-LK1.png").unwrap().to_luma8();
    let img2 = open("./assets/ch8-LK2.png").unwrap().to_luma8();

    // 将 ImageBuffer<Luma<u8>, Vec<u8>> 转换为 DynamicImage
    let img1_dynamic = DynamicImage::ImageLuma8(img1.clone());
    let img2_dynamic = DynamicImage::ImageLuma8(img2.clone());

    // 设置关键点数量
    let n_keypoints = 500;

    // 第一步: 检测Oriented FAST角点位置并计算BRIEF描述子
    let start_time = Instant::now();
    let img1_keypoints = orb::orb(&img1_dynamic, n_keypoints).unwrap();
    let img2_keypoints = orb::orb(&img2_dynamic, n_keypoints).unwrap();
    let end_time = Instant::now();
    println!("提取ORB特征点耗时: {:?} 秒", end_time - start_time);

    // 将关键点转换为 Vector2<f64> 格式
    let kp1: Vec<Vector2<f64>> = img1_keypoints.iter()
        .map(|kp| Vector2::new(kp.x as f64, kp.y as f64))
        .collect();
    let mut kp2 = Vec::new();
    let mut success = Vec::new();

    /* start 单层光流追踪 */
    println!("单层光流追踪\n");
    // 单层光流追踪
    optical_flow_single_level(&img1, &img2, &kp1, &mut kp2, &mut success, false, false);
    // 保存结果图片
    let mut img2_rgb: RgbImage = img2.convert();
    for (i, &s) in success.iter().enumerate() {
        if s {
            let start = (kp1[i].x as f32, kp1[i].y as f32);
            let end = (kp2[i].x as f32, kp2[i].y as f32);
            draw_line_segment_mut(&mut img2_rgb, start, end, Rgb([0, 255, 0]));
            // 修复 draw_cross_mut 调用
            let (x, y) = (kp2[i].x as i32, kp2[i].y as i32); // 转换为 i32
            draw_cross_mut(&mut img2_rgb, Rgb([0, 255, 0]), x, y);
        }
    }
    img2_rgb.save("./ch8-LK2_result_single_level.png").unwrap();

    for (i, &s) in success.iter().enumerate() {
        if s {
            println!("Keypoint {} tracked successfully to {:?}", i, kp2[i]);
        } else {
            println!("Keypoint {} tracking failed", i);
        }
    }
    /* end 单层光流追踪 */

    /* start 多层光流追踪 */
    println!("多层光流追踪\n");
    // 多层光流追踪
    optical_flow_multi_level(&img1, &img2, &kp1, &mut kp2, &mut success, false);

    // 保存结果图片
    let mut img2_rgb: RgbImage = img2.clone().convert();
    for (i, &s) in success.iter().enumerate() {
        if s {
            let start = (kp1[i].x as f32, kp1[i].y as f32);
            let end = (kp2[i].x as f32, kp2[i].y as f32);
            draw_line_segment_mut(&mut img2_rgb, start, end, Rgb([0, 255, 0]));
            // 修复 draw_cross_mut 调用
            let (x, y) = (kp2[i].x as i32, kp2[i].y as i32); // 转换为 i32
            draw_cross_mut(&mut img2_rgb, Rgb([0, 255, 0]), x, y);
        }
    }
    img2_rgb.save("./ch8-LK2_result_multi_level.png").unwrap();

    for (i, &s) in success.iter().enumerate() {
        if s {
            println!("Keypoint {} tracked successfully to {:?}", i, kp2[i]);
        } else {
            println!("Keypoint {} tracking failed", i);
        }
    }
    /* end 多层光流追踪 */
}