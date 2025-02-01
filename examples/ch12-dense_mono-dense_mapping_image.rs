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

//! 本程序演示了单目相机在已知轨迹下的稠密深度估计，
//! 使用极线搜索 + NCC 匹配的方式，与书本的 12.2 节对应。
//! 添加了图像保存到文件的功能。

use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::fs::create_dir_all;

use std::cell::{
    RefMut, Ref, RefCell
};

// 线性代数
use nalgebra::{
    Quaternion, Vector3, Matrix3, UnitQuaternion, 
    Isometry3, Translation3, Const,
    Matrix, Point3, ViewStorage, Rotation3,
    Matrix3x1, VectorView3, DMatrix, DVector, SVector,
    Vector6, Matrix6, Vector2, Matrix2, 
};

// 李代数
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

// 图像处理
use image::{
    DynamicImage, GrayImage, ImageBuffer, Luma, 
    GenericImageView, Rgb, RgbImage, 
    buffer::ConvertBuffer,
};
use imageproc::drawing::{
    draw_cross_mut, draw_filled_circle_mut, draw_line_segment_mut,
};

// 随机数
use rand::Rng;
use rand_distr::{
    Distribution, Normal
};

// 边缘宽度
const BOARDER: i32 = 20;     
// 图像宽度    
const WIDTH: i32 = 640;          
// 图像高度
const HEIGHT: i32 = 480;         
// 相机内参
const FX: f64 = 481.2;           
const FY: f64 = -480.0;
const CX: f64 = 319.5;
const CY: f64 = 239.5;
// NCC 取的窗口半宽度
const NCC_WINDOW_SIZE: i32 = 3;  
// NCC窗口面积
const NCC_AREA: i32 = (2 * NCC_WINDOW_SIZE + 1) * (2 * NCC_WINDOW_SIZE + 1); 
// 收敛判定：最小方差
const MIN_COV: f64 = 0.1;        
// 发散判定：最大方差
const MAX_COV: f64 = 10.0;    
// 图像保存目录
const OUTPUT_DIR: &str = "./result/ch12-dense_mono_dense_mapping";

// 从 REMODE 数据集读取数据
fn read_dataset_files(
    path: &str,
    color_image_files: &mut Vec<String>,
    poses: &mut Vec<SE3>,
    ref_depth: &mut ImageBuffer<Luma<f64>, Vec<f64>>,
) -> io::Result<()> {
    println!("开始读取数据集文件...");
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
        poses.push(SE3::from_rot_trans(
            SO3::from_xyzw(data[3], data[4], data[5],data[6]),
            Vector3::new(data[0], data[1], data[2]),
        ));
    }

    println!("读取轨迹和图像文件完成，共读取 {} 张图像", color_image_files.len());

    // 加载参考深度图
    let depth_file = File::open(format!("{}/depthmaps/scene_000.depth", path))?;
    let depth_reader = io::BufReader::new(depth_file);

    for (y, line) in depth_reader.lines().enumerate() {
        let line = line?;
        let depths: Vec<f64> = line.split_whitespace().map(|s| s.parse().unwrap()).collect();
        for (x, depth) in depths.iter().enumerate() {
            let x_index = std::cmp::min(ref_depth.width() as usize - 1, x);
            let y_index = std::cmp::min(ref_depth.height() as usize - 1, y);
            ref_depth.put_pixel(x_index as u32, y_index as u32, Luma([*depth / 100.0]));
        }
    }

    println!("参考深度图加载完成");
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
    println!("开始极线搜索，参考点：{:?}", pt_ref);
    let f_ref = px2cam(pt_ref);
    let p_ref = f_ref * depth_mu;

    let px_mean_curr = cam2px(&(t_c_r.apply((&p_ref).into())));
    let d_min = depth_mu - 3.0 * depth_cov;
    let d_max = depth_mu + 3.0 * depth_cov;
    let px_min_curr = cam2px(&(t_c_r.apply((&(f_ref * d_min)).into())));
    let px_max_curr = cam2px(&(t_c_r.apply((&(f_ref * d_max)).into())));

    let epipolar_line = px_max_curr - px_min_curr;
    *epipolar_direction = epipolar_line;
    epipolar_direction.normalize();
    let half_length = 0.5 * epipolar_line.norm();

    // 在极线上搜索，以深度均值点为中心，左右各取半长度
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
        println!("极线搜索失败，最佳 NCC 为 {}", best_ncc);
        return false;
    }

    *pt_curr = best_px_curr;
    println!("极线搜索成功，最佳匹配点：{:?}", pt_curr);
    true
}

// 计算 NCC 评分
fn ncc(ref_img: &GrayImage, curr_img: &GrayImage, pt_ref: &Vector2<f64>, pt_curr: &Vector2<f64>) -> f64 {
    println!("计算 NCC 评分，参考点：{:?}，当前点：{:?}", pt_ref, pt_curr);
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

    let ncc_score = numerator / (demoniator1 * demoniator2 + 1e-10).sqrt();
    println!("NCC 评分：{}", ncc_score);
    ncc_score
}

// 双线性灰度插值
fn get_bilinear_interpolated_value(img: &GrayImage, pt: &Vector2<f64>) -> f64 {
    println!("双线性插值，点：{:?}", pt);
    let x = pt.x as u32;
    let y = pt.y as u32;
    let xx = pt.x - x as f64;
    let yy = pt.y - y as f64;

    let d00 = img.get_pixel(x, y)[0] as f64;
    let d01 = img.get_pixel(x, y + 1)[0] as f64;
    let d10 = img.get_pixel(x + 1, y)[0] as f64;
    let d11 = img.get_pixel(x + 1, y + 1)[0] as f64;

    let interpolated_value = ((1.0 - xx) * (1.0 - yy) * d00 + xx * (1.0 - yy) * d10 + (1.0 - xx) * yy * d01 + xx * yy * d11) / 255.0;
    println!("插值结果：{}", interpolated_value);
    interpolated_value
}

// 像素到相机坐标系
fn px2cam(px: &Vector2<f64>) -> Vector3<f64> {
    println!("像素到相机坐标系，像素点：{:?}", px);
    Vector3::new((px.x - CX) / FX, (px.y - CY) / FY, 1.0)
}

// 相机坐标系到像素
fn cam2px(p_cam: &Vector3<f64>) -> Vector2<f64> {
    println!("相机坐标系到像素，相机点：{:?}", p_cam);
    Vector2::new(p_cam.x * FX / p_cam.z + CX, p_cam.y * FY / p_cam.z + CY)
}

// 检测一个点是否在图像边框内
fn inside(pt: &Vector2<f64>) -> bool {
    println!("检查点是否在图像边框内，点：{:?}", pt);
    pt.x >= (BOARDER as f64) && pt.y >= (BOARDER as f64) && pt.x + (BOARDER as f64) < (WIDTH as f64) && pt.y + (BOARDER as f64) < (HEIGHT as f64)
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
    println!("更新深度滤波器，参考点：{:?}，当前点：{:?}", pt_ref, pt_curr);
    let t_r_c = t_c_r.inverse();
    let f_ref = px2cam(pt_ref).normalize();
    let f_curr = px2cam(pt_curr).normalize();

    let t = t_r_c.xyz().clone();
    let f2 = t_r_c.rot().apply((&f_curr).into());
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

    println!("深度更新完成，新深度：{}", mu_fuse);
    true
}

// 显示并保存极线匹配
fn show_epipolar_match(ref_img: &GrayImage, curr_img: &GrayImage, px_ref: (i32, i32), px_curr: (i32, i32), index: usize, output_dir: &Path) {
    let mut ref_show : RgbImage = ref_img.convert();
    let mut curr_show : RgbImage = curr_img.convert();

    draw_filled_circle_mut(&mut ref_show, px_ref, 5, Rgb([0, 255, 0]));
    draw_filled_circle_mut(&mut curr_show, px_curr, 5, Rgb([255, 0, 0]));

    let rand_number = rand::random::<u32>() % 1000;
    ref_show.save(output_dir.join(format!("ref_match_{index}_{rand_number}.png"))).unwrap();
    curr_show.save(output_dir.join(format!("curr_match_{index}_{rand_number}.png"))).unwrap();
}

// 显示并保存极线
fn show_epipolar_line(ref_img: &GrayImage, curr_img: &GrayImage, px_ref: (f32, f32), px_min_curr: (f32, f32), px_max_curr: (f32, f32), index: usize, output_dir: &Path) {
    let mut curr_show : RgbImage = curr_img.convert();

    draw_line_segment_mut(&mut curr_show, px_min_curr, px_max_curr, Rgb([0, 255, 0])); //极线
    draw_filled_circle_mut(&mut curr_show, (px_ref.0 as i32, px_ref.1 as i32), 5, Rgb([255, 0, 0])); // 参考点
    let rand_number = rand::random::<u32>() % 1000;
    curr_show.save(output_dir.join(format!("epipolar_line_{index}_{rand_number}.png"))).unwrap();
}

// 评估深度估计
fn evaluate_depth(depth_truth: &ImageBuffer<Luma<f64>, Vec<f64>>, depth_estimate: &ImageBuffer<Luma<f64>, Vec<f64>>) {
    let mut error_sum = 0.0;
    let mut error_sq_sum = 0.0;
    let mut count = 0;

    for (x, y, pixel) in depth_truth.enumerate_pixels() {
        let est_pixel = depth_estimate.get_pixel(x, y)[0];
        let error = pixel[0] - est_pixel;
        error_sum += error;
        error_sq_sum += error * error;
        count += 1;
    }

    let avg_error = error_sum / count as f64;
    let avg_sq_error = error_sq_sum / count as f64;

    println!("Average Error: {avg_error}, Average Squared Error: {avg_sq_error}");
}

// 保存深度图可视化
fn save_depth_image(depth_image: &ImageBuffer<Luma<f64>, Vec<f64>>, file_path: &Path) {
    let max_depth = depth_image.pixels().map(|p| p[0]).fold(f64::MIN, f64::max);
    let scale_factor = 255.0 / max_depth;
    let buffer: Vec<u8> = depth_image
        .pixels()
        .map(|p| (p[0] * scale_factor) as u8)
        .collect();
    let new_img = GrayImage::from_raw(depth_image.width(), depth_image.height(), buffer).unwrap();
    new_img.save(file_path).unwrap();
}

// 主函数
fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("程序开始执行...");
    // 判断图像保存目录是否存在
    let output_dir = Path::new(OUTPUT_DIR);
    if !output_dir.exists() {
        create_dir_all(output_dir)?;
    }

    // 加载数据集
    let path = "./assets/ch12-REMODE";
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
        println!("处理第 {} 张图像", index);
        let curr_img = image::open(&color_image_files[index])?.to_luma8();
        let pose_curr_twc = &poses[index];
        let pose_t_c_r = pose_curr_twc.inverse() * pose_ref_twc.clone();

        // 更新深度图
        for x in BOARDER..WIDTH - BOARDER {
            for y in BOARDER..HEIGHT - BOARDER {
                let depth_cov = (depth_cov2.get_pixel(x as u32, y as u32)[0] as f64).sqrt();
                if depth_cov < MIN_COV || depth_cov > MAX_COV {
                    continue;
                }

                let mut pt_curr = Vector2::zeros();
                let mut epipolar_direction = Vector2::zeros();
                let pt_ref = Vector2::new(x as f64, y as f64);

                if epipolar_search(&ref_img, &curr_img, &pose_t_c_r, &pt_ref, depth.get_pixel(x as u32, y as u32)[0], depth_cov, &mut pt_curr, &mut epipolar_direction) {
                    update_depth_filter(&pt_ref, &pt_curr, &pose_t_c_r, &epipolar_direction, &mut depth, &mut depth_cov2);
                    // 显示极线(线段)
                    show_epipolar_line(&ref_img, &curr_img, (x as f32, y as f32), (pt_curr.x as f32, pt_curr.y as f32), (pt_curr.x as f32 + epipolar_direction.x as f32, pt_curr.y as f32 + epipolar_direction.y as f32), index, output_dir);
                    // 显示极线匹配
                    show_epipolar_match(&ref_img, &curr_img, (x, y), (pt_curr.x as i32, pt_curr.y as i32), index + 1, output_dir);
                }
            }
        }
        evaluate_depth(&ref_depth, &depth);
        let rand_number = rand::random::<u32>() % 1000;
        save_depth_image(&depth, &output_dir.join(format!("depth_{index}_{rand_number}.png")));
    }

    println!("程序执行完成");
    Ok(())
}