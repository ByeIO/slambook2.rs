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
#![allow(unused_doc_comments)]

//! 本程序演示了单目相机在已知轨迹下的稠密深度估计,
//! 使用极线搜索 + NCC 匹配的方式，与书本的 12.2 节对应.

use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;

use std::cell::{
    RefMut, Ref, RefCell
};

use nalgebra::{
    Quaternion, Vector3, Matrix3, UnitQuaternion, 
    Isometry3, Translation3, Const,
    Matrix, Point3, ViewStorage, Rotation3,
    Matrix3x1, VectorView3, DMatrix, DVector, SVector,
    Vector6, Matrix6
};

use factrs::{
    assign_symbols,
    core::{BetweenResidual, GaussNewton, Graph, Values},
    dtype, fac,
    linalg::{ ForwardProp, Numeric, NumericalDiff, VectorX, DiffResult, MatrixX },
    residuals::{Residual1, Residual2},
    traits::*,
    variables::{VectorVar2, SE2, VectorVar3, SE3, SO3, SO2, MatrixLieGroup},
    containers::Key,
    noise::{GaussianNoise},
    optimizers::{LevenMarquardt}
};

use image::{
    DynamicImage, GrayImage, ImageBuffer, Luma
};
use imageproc::drawing::draw_cross_mut;

// 参数
const BOARDER: i32 = 20;         // 边缘宽度
const WIDTH: i32 = 640;          // 图像宽度
const HEIGHT: i32 = 480;         // 图像高度
const FX: f64 = 481.2;           // 相机内参
const FY: f64 = -480.0;
const CX: f64 = 319.5;
const CY: f64 = 239.5;
const NCC_WINDOW_SIZE: i32 = 3;  // NCC 取的窗口半宽度
const NCC_AREA: i32 = (2 * NCC_WINDOW_SIZE + 1) * (2 * NCC_WINDOW_SIZE + 1); // NCC窗口面积
const MIN_COV: f64 = 0.1;        // 收敛判定：最小方差
const MAX_COV: f64 = 10.0;       // 发散判定：最大方差

// 从 REMODE 数据集读取数据
fn read_dataset_files(
    path: &str,
    color_image_files: &mut Vec<String>,
    poses: &mut Vec<SE3>,
    ref_depth: &mut ImageBuffer<Luma<f64>, Vec<f64>>,
) -> io::Result<()> {
    let file = File::open(format!("{}/first_200_frames_traj_over_table_input_sequence.txt", path))?;
    let reader = io::BufReader::new(file);

    for line in reader.lines() {
        let line = line?;
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() < 8 {
            continue;
        }

        let image = parts[0];
        let data: Vec<f64> = parts[1..8].iter().map(|s| s.parse().unwrap()).collect();

        color_image_files.push(format!("{}/images/{}", path, image));
        poses.push(SE3::from_quaternion_and_translation(
            Quaternion::new(data[6], data[3], data[4], data[5]),
            Vector3::new(data[0], data[1], data[2]),
        ));
    }

    // 加载参考深度图
    let depth_file = File::open(format!("{}/depthmaps/scene_000.depth", path))?;
    let depth_reader = io::BufReader::new(depth_file);

    for (y, line) in depth_reader.lines().enumerate() {
        let line = line?;
        let depths: Vec<f64> = line.split_whitespace().map(|s| s.parse().unwrap()).collect();
        for (x, depth) in depths.iter().enumerate() {
            ref_depth.put_pixel(x as u32, y as u32, Luma([*depth / 100.0]));
        }
    }

    Ok(())
}

// 极线搜索
fn epipolar_search(
    ref_img: &GrayImage,
    curr_img: &GrayImage,
    t_c_r: &SE3,
    pt_ref: &Vector2<f64>,
    depth_mu: f64,
    depth_cov: f64,
    pt_curr: &mut Vector2<f64>,
    epipolar_direction: &mut Vector2<f64>,
) -> bool {
    let f_ref = px2cam(pt_ref);
    let p_ref = f_ref * depth_mu;

    let px_mean_curr = cam2px(&(t_c_r * p_ref));
    let d_min = depth_mu - 3.0 * depth_cov;
    let d_max = depth_mu + 3.0 * depth_cov;
    let px_min_curr = cam2px(&(t_c_r * (f_ref * d_min)));
    let px_max_curr = cam2px(&(t_c_r * (f_ref * d_max)));

    let epipolar_line = px_max_curr - px_min_curr;
    *epipolar_direction = epipolar_line;
    epipolar_direction.normalize();
    let half_length = 0.5 * epipolar_line.norm();

    let mut best_ncc = -1.0;
    let mut best_px_curr = Vector2::zeros();

    for l in (-half_length as i32..=half_length as i32).step_by(1) {
        let px_curr = px_mean_curr + (l as f64) * *epipolar_direction;
        if !inside(&px_curr) {
            continue;
        }

        let ncc = ncc(ref_img, curr_img, pt_ref, &px_curr);
        if ncc > best_ncc {
            best_ncc = ncc;
            best_px_curr = px_curr;
        }
    }

    if best_ncc < 0.85 {
        return false;
    }

    *pt_curr = best_px_curr;
    true
}

// 计算 NCC 评分
fn ncc(ref_img: &GrayImage, curr_img: &GrayImage, pt_ref: &Vector2<f64>, pt_curr: &Vector2<f64>) -> f64 {
    let mut mean_ref = 0.0;
    let mut mean_curr = 0.0;
    let mut values_ref = Vec::new();
    let mut values_curr = Vec::new();

    for x in -NCC_WINDOW_SIZE..=NCC_WINDOW_SIZE {
        for y in -NCC_WINDOW_SIZE..=NCC_WINDOW_SIZE {
            let value_ref = ref_img.get_pixel((pt_ref.x + x as f64) as u32, (pt_ref.y + y as f64) as u32)[0] as f64 / 255.0;
            mean_ref += value_ref;

            let value_curr = get_bilinear_interpolated_value(curr_img, &(pt_curr + Vector2::new(x as f64, y as f64)));
            mean_curr += value_curr;

            values_ref.push(value_ref);
            values_curr.push(value_curr);
        }
    }

    mean_ref /= NCC_AREA as f64;
    mean_curr /= NCC_AREA as f64;

    let mut numerator = 0.0;
    let mut demoniator1 = 0.0;
    let mut demoniator2 = 0.0;

    for i in 0..values_ref.len() {
        let n = (values_ref[i] - mean_ref) * (values_curr[i] - mean_curr);
        numerator += n;
        demoniator1 += (values_ref[i] - mean_ref) * (values_ref[i] - mean_ref);
        demoniator2 += (values_curr[i] - mean_curr) * (values_curr[i] - mean_curr);
    }

    numerator / (demoniator1 * demoniator2 + 1e-10).sqrt()
}

// 双线性灰度插值
fn get_bilinear_interpolated_value(img: &GrayImage, pt: &Vector2<f64>) -> f64 {
    let x = pt.x as u32;
    let y = pt.y as u32;
    let xx = pt.x - x as f64;
    let yy = pt.y - y as f64;

    let d00 = img.get_pixel(x, y)[0] as f64;
    let d01 = img.get_pixel(x, y + 1)[0] as f64;
    let d10 = img.get_pixel(x + 1, y)[0] as f64;
    let d11 = img.get_pixel(x + 1, y + 1)[0] as f64;

    ((1.0 - xx) * (1.0 - yy) * d00 + xx * (1.0 - yy) * d10 + (1.0 - xx) * yy * d01 + xx * yy * d11) / 255.0
}

// 像素到相机坐标系
fn px2cam(px: &Vector2<f64>) -> Vector3<f64> {
    Vector3::new((px.x - CX) / FX, (px.y - CY) / FY, 1.0)
}

// 相机坐标系到像素
fn cam2px(p_cam: &Vector3<f64>) -> Vector2<f64> {
    Vector2::new(p_cam.x * FX / p_cam.z + CX, p_cam.y * FY / p_cam.z + CY)
}

// 检测一个点是否在图像边框内
fn inside(pt: &Vector2<f64>) -> bool {
    pt.x >= BOARDER as f64 && pt.y >= BOARDER as f64 && pt.x + BOARDER as f64 < WIDTH as f64 && pt.y + BOARDER as f64 < HEIGHT as f64
}

// 更新深度滤波器
fn update_depth_filter(
    pt_ref: &Vector2<f64>,
    pt_curr: &Vector2<f64>,
    t_c_r: &SE3,
    epipolar_direction: &Vector2<f64>,
    depth: &mut ImageBuffer<Luma<f64>, Vec<f64>>,
    depth_cov2: &mut ImageBuffer<Luma<f64>, Vec<f64>>,
) -> bool {
    let t_r_c = t_c_r.inverse();
    let f_ref = px2cam(pt_ref).normalize();
    let f_curr = px2cam(pt_curr).normalize();

    let t = t_r_c.translation();
    let f2 = t_r_c.so3() * f_curr;
    let b = Vector2::new(t.dot(&f_ref), t.dot(&f2));
    let a = Matrix2::new(
        f_ref.dot(&f_ref),
        -f_ref.dot(&f2),
        -f_ref.dot(&f2),
        -f2.dot(&f2),
    );
    let ans = a.try_inverse().unwrap() * b;
    let xm = ans[0] * f_ref;
    let xn = t + ans[1] * f2;
    let p_esti = (xm + xn) / 2.0;
    let depth_estimation = p_esti.norm();

    // 计算不确定性
    let p = f_ref * depth_estimation;
    let a = p - t;
    let t_norm = t.norm();
    let a_norm = a.norm();
    let alpha = (f_ref.dot(&t) / t_norm).acos();
    let beta = (-a.dot(&t) / (a_norm * t_norm)).acos();
    let f_curr_prime = px2cam(&(pt_curr + epipolar_direction)).normalize();
    let beta_prime = (f_curr_prime.dot(&-t) / t_norm).acos();
    let gamma = std::f64::consts::PI - alpha - beta_prime;
    let p_prime = t_norm * beta_prime.sin() / gamma.sin();
    let d_cov = p_prime - depth_estimation;
    let d_cov2 = d_cov * d_cov;

    // 高斯融合
    let mu = depth.get_pixel(pt_ref.x as u32, pt_ref.y as u32)[0];
    let sigma2 = depth_cov2.get_pixel(pt_ref.x as u32, pt_ref.y as u32)[0];

    let mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
    let sigma_fuse2 = (sigma2 * d_cov2) / (sigma2 + d_cov2);

    depth.put_pixel(pt_ref.x as u32, pt_ref.y as u32, Luma([mu_fuse]));
    depth_cov2.put_pixel(pt_ref.x as u32, pt_ref.y as u32, Luma([sigma_fuse2]));

    true
}

// 主函数
fn main() -> io::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        println!("Usage: dense_mapping path_to_test_dataset");
        return Ok(());
    }

    let path = &args[1];
    let mut color_image_files = Vec::new();
    let mut poses = Vec::new();
    let mut ref_depth = ImageBuffer::new(WIDTH as u32, HEIGHT as u32);

    read_dataset_files(path, &mut color_image_files, &mut poses, &mut ref_depth)?;

    let ref_img = image::open(&color_image_files[0])?.to_luma8();
    let pose_ref_twc = &poses[0];
    let init_depth = 3.0;
    let init_cov2 = 3.0;
    let mut depth = ImageBuffer::from_pixel(WIDTH as u32, HEIGHT as u32, Luma([init_depth]));
    let mut depth_cov2 = ImageBuffer::from_pixel(WIDTH as u32, HEIGHT as u32, Luma([init_cov2]));

    for index in 1..color_image_files.len() {
        let curr_img = image::open(&color_image_files[index])?.to_luma8();
        let pose_curr_twc = &poses[index];
        let pose_t_c_r = pose_curr_twc.inverse() * pose_ref_twc;

        // 更新深度图
        for x in BOARDER..WIDTH - BOARDER {
            for y in BOARDER..HEIGHT - BOARDER {
                let depth_cov = depth_cov2.get_pixel(x as u32, y as u32)[0].sqrt();
                if depth_cov < MIN_COV || depth_cov > MAX_COV {
                    continue;
                }

                let mut pt_curr = Vector2::zeros();
                let mut epipolar_direction = Vector2::zeros();
                let pt_ref = Vector2::new(x as f64, y as f64);

                if epipolar_search(&ref_img, &curr_img, &pose_t_c_r, &pt_ref, depth.get_pixel(x as u32, y as u32)[0], depth_cov, &mut pt_curr, &mut epipolar_direction) {
                    update_depth_filter(&pt_ref, &pt_curr, &pose_t_c_r, &epipolar_direction, &mut depth, &mut depth_cov2);
                }
            }
        }
    }

    Ok(())
}