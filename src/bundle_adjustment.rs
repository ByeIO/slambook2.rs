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

// 标准库
use std::time::Instant;
use std::cell::{
    RefMut, Ref, RefCell
};
use std::f64::consts::PI;
use std::f64::EPSILON;
use std::process::exit;
use std::ops::Add;
use std::ops::Mul;
use std::fs::File;
use std::io::{self, BufWriter, Write, BufReader, BufRead};
use std::path::Path;
use std::vec::Vec;

// 内部库
use crate::common::{
    rand_normal, angle_axis_to_quaternion, quaternion_to_angle_axis, angle_axis_rotate_point
};

// 定义符号变量
assign_symbols!(_X_: VectorVar3);
assign_symbols!(_Y_: SE2);
assign_symbols!(_Z_: SE3);

/* start Snavely 重投影误差 */

// 定义SnavelyReprojectionError结构体
#[derive(Clone, Debug)]
pub struct SnavelyReprojectionError {
    observed_x: f64,
    observed_y: f64,
}

impl SnavelyReprojectionError {
    pub fn new(observed_x: f64, observed_y: f64) -> Self {
        Self { observed_x, observed_y }
    }

    // 相机投影函数，带有畸变
    fn cam_projection_with_distortion<T: Numeric>(camera: &[T; 9], point: &[T; 3], predictions: &mut [T; 2]) {
        // Rodrigues' formula
        let mut p = [T::from_f64(0.0).unwrap(), T::from_f64(0.0).unwrap(), T::from_f64(0.0).unwrap()];
        Self::angle_axis_rotate_point(&camera[0..3], point, &mut p);

        // 平移
        p[0] = p[0] + camera[3];
        p[1] = p[1] + camera[4];
        p[2] = p[2] + camera[5];

        // 计算畸变中心
        let xp = -p[0] / p[2];
        let yp = -p[1] / p[2];

        // 应用二阶和四阶径向畸变
        let l1 = camera[7];
        let l2 = camera[8];

        let r2 = xp * xp + yp * yp;
        let distortion = T::from_f64(1.0).unwrap() + r2 * (l1 + l2 * r2);

        let focal = camera[6];
        predictions[0] = focal * distortion * xp;
        predictions[1] = focal * distortion * yp;
    }

    // 角度轴旋转点
    fn angle_axis_rotate_point<T: Numeric>(angle_axis: &[T], point: &[T; 3], result: &mut [T; 3]) {
        let theta2 = angle_axis[0] * angle_axis[0] + angle_axis[1] * angle_axis[1] + angle_axis[2] * angle_axis[2];
        if theta2 > T::from_f64(0.0).unwrap() {
            let theta = theta2.sqrt();
            let costheta = theta.cos();
            let sintheta = theta.sin();
            let theta_inverse = T::from_f64(1.0).unwrap() / theta;

            let w = [angle_axis[0] * theta_inverse, angle_axis[1] * theta_inverse, angle_axis[2] * theta_inverse];
            let w_cross_pt = [
                w[1] * point[2] - w[2] * point[1],
                w[2] * point[0] - w[0] * point[2],
                w[0] * point[1] - w[1] * point[0],
            ];
            let tmp = (w[0] * point[0] + w[1] * point[1] + w[2] * point[2]) * (T::from_f64(1.0).unwrap() - costheta);

            result[0] = point[0] * costheta + w_cross_pt[0] * sintheta + w[0] * tmp;
            result[1] = point[1] * costheta + w_cross_pt[1] * sintheta + w[1] * tmp;
            result[2] = point[2] * costheta + w_cross_pt[2] * sintheta + w[2] * tmp;
        } else {
            result[0] = point[0];
            result[1] = point[1];
            result[2] = point[2];
        }
    }
}

// 实现 Residual1 trait 用于因子
#[factrs::mark]
impl Residual1 for SnavelyReprojectionError {
    type Differ = ForwardProp<<Self as Residual1>::DimIn>;
    type V1 = VectorVar3;
    type DimIn = Const<9>;
    type DimOut = Const<2>;

    fn residual1<T: Numeric>(&self, camera: VectorVar3<T>) -> VectorX<T> {
        let point = VectorVar3::new(T::from_f64(0.0).unwrap(), T::from_f64(0.0).unwrap(), T::from_f64(0.0).unwrap());

        // 将 VectorVar3 转换为数组
        // let camera_arr = [camera[0], camera[1], camera[2], camera[3], camera[4], camera[5], camera[6], camera[7], camera[8]];
        let camera_arr = [camera[0], camera[1], camera[2], camera[0], camera[1], camera[2], camera[0], camera[1], camera[2]];
        let point_arr = [point[0], point[1], point[2]];

        let mut predictions = [T::from_f64(0.0).unwrap(), T::from_f64(0.0).unwrap()];
        Self::cam_projection_with_distortion(&camera_arr, &point_arr, &mut predictions);

        let error_x = predictions[0] - T::from_f64(self.observed_x).unwrap();
        let error_y = predictions[1] - T::from_f64(self.observed_y).unwrap();

        VectorX::from_column_slice(&[error_x, error_y])
    }
}

/* end Snavely 重投影误差 */

/* start BAL问题 */

/// 从文件读入BAL dataset
pub struct BALProblem {
    num_cameras: i32,
    num_points: i32,
    num_observations: i32,
    num_parameters: i32,
    use_quaternions: bool,

    point_index: Vec<i32>,
    camera_index: Vec<i32>,
    observations: Vec<f64>,
    parameters: RefCell<Vec<f64>>, // 使用 RefCell 包装 parameters
}

impl Clone for BALProblem {
    fn clone(&self) -> Self {
        BALProblem {
            num_cameras: self.num_cameras,
            num_points: self.num_points,
            num_observations: self.num_observations,
            num_parameters: self.num_parameters,
            use_quaternions: self.use_quaternions,
            point_index: self.point_index.clone(),
            camera_index: self.camera_index.clone(),
            observations: self.observations.clone(),
            parameters: RefCell::new(self.parameters.borrow().clone()), // 克隆 RefCell 内部的数据
        }
    }
}

impl BALProblem {
    /// 从文本文件加载BAL数据
    pub fn new(filename: &str, use_quaternions: bool) -> Self {
        // 打开文件
        let mut file = File::open(filename).expect("Unable to open file");
        let mut reader = BufReader::new(file);

        let mut num_cameras = 0;
        let mut num_points = 0;
        let mut num_observations = 0;

        // 读取头信息
        let mut line = String::new();
        reader.read_line(&mut line).expect("Unable to read header");
        let header: Vec<&str> = line.trim().split_whitespace().collect();
        num_cameras = header[0].parse().unwrap();
        num_points = header[1].parse().unwrap();
        num_observations = header[2].parse().unwrap();

        let mut point_index = vec![0; num_observations as usize];
        let mut camera_index = vec![0; num_observations as usize];
        let mut observations = vec![0.0; 2 * num_observations as usize];

        let num_parameters = 9 * num_cameras + 3 * num_points;
        let mut parameters = vec![0.0; num_parameters as usize];

        // 读取观测数据
        for i in 0..num_observations {
            let mut line = String::new();
            reader.read_line(&mut line).expect("Unable to read observation");
            let obs: Vec<&str> = line.trim().split_whitespace().collect();
            camera_index[i as usize] = obs[0].parse().unwrap();
            point_index[i as usize] = obs[1].parse().unwrap();
            observations[2 * i as usize] = obs[2].parse().unwrap();
            observations[2 * i as usize + 1] = obs[3].parse().unwrap();
        }

        // 读取参数数据
        for i in 0..num_parameters {
            let mut line = String::new();
            reader.read_line(&mut line).expect("Unable to read parameter");
            parameters[i as usize] = line.trim().parse().unwrap();
        }

        if use_quaternions {
            // 将角轴转换为四元数
            let num_parameters = 10 * num_cameras + 3 * num_points;
            let mut quaternion_parameters = vec![0.0; num_parameters as usize];
            let mut original_cursor = 0;
            let mut quaternion_cursor = 0;

            for _ in 0..num_cameras {

                let angle_axis_arr: [f64; 3] = [parameters[original_cursor], parameters[original_cursor + 1], parameters[original_cursor + 2]];
                let mut quaternion_arr: [f64; 4] = [0.0; 4];
                angle_axis_to_quaternion(&angle_axis_arr, &mut quaternion_arr);

                quaternion_cursor += 4;
                original_cursor += 3;
                for j in 4..10 {
                    quaternion_parameters[quaternion_cursor] = parameters[original_cursor];
                    quaternion_cursor += 1;
                    original_cursor += 1;
                }
            }

            // 复制剩余的点数据
            for i in 0..3 * num_points {
                quaternion_parameters[quaternion_cursor] = parameters[original_cursor];
                quaternion_cursor += 1;
                original_cursor += 1;
            }

            parameters = quaternion_parameters;
        }

        BALProblem {
            num_cameras,
            num_points,
            num_observations,
            num_parameters,
            use_quaternions,
            point_index,
            camera_index,
            observations,
            parameters: RefCell::new(parameters), // 使用 RefCell 包装 parameters
        }
    }

    /// 将结果保存到文本文件
    pub fn write_to_file(&self, filename: &str) -> io::Result<()> {
        let mut file = BufWriter::new(File::create(filename)?);

        writeln!(file, "{} {} {} {}", self.num_cameras, self.num_cameras, self.num_points, self.num_observations)?;

        for i in 0..self.num_observations {
            write!(file, "{} {}", self.camera_index[i as usize], self.point_index[i as usize])?;
            for j in 0..2 {
                write!(file, " {}", self.observations[2 * i as usize + j])?;
            }
            writeln!(file)?;
        }

        let parameters = self.parameters.borrow(); // 获取不可变引用
        for i in 0..self.num_cameras() {
            let mut angleaxis = [0.0; 9];
            if self.use_quaternions {
                let quaternion: [f64; 4] = [
                    parameters[10 * i as usize],
                    parameters[10 * i as usize + 1],
                    parameters[10 * i as usize + 2],
                    parameters[10 * i as usize + 3],
                ];
                let mut angle_axis_arr: [f64; 3] = [0.0; 3];
                quaternion_to_angle_axis(&quaternion, &mut angle_axis_arr);
                angleaxis[0..3].copy_from_slice(&angle_axis_arr);
                angleaxis[3..9].copy_from_slice(&parameters[10 * i as usize + 4..10 * i as usize + 10]);
            } else {
                angleaxis[0..9].copy_from_slice(&parameters[9 * i as usize..9 * i as usize + 9]);
            }
            for j in 0..9 {
                writeln!(file, "{:.16}", angleaxis[j])?;
            }
        }

        let points = &parameters[self.camera_block_size() as usize * self.num_cameras as usize..];
        for i in 0..self.num_points() {
            let point = &points[i as usize * self.point_block_size() as usize..];
            for j in 0..self.point_block_size() {
                writeln!(file, "{:.16}", point[j as usize])?;
            }
        }

        Ok(())
    }

    /// 将问题保存为PLY文件以便在Meshlab或CloudCompare中检查
    pub fn write_to_ply_file(&self, filename: &str) -> io::Result<()> {
        let mut file = BufWriter::new(File::create(filename)?);

        writeln!(file, "ply")?;
        writeln!(file, "format ascii 1.0")?;
        writeln!(file, "element vertex {}", self.num_cameras + self.num_points)?;
        writeln!(file, "property float x")?;
        writeln!(file, "property float y")?;
        writeln!(file, "property float z")?;
        writeln!(file, "property uchar red")?;
        writeln!(file, "property uchar green")?;
        writeln!(file, "property uchar blue")?;
        writeln!(file, "end_header")?;

        let parameters = self.parameters.borrow(); // 获取不可变引用

        // 导出外参数据（即相机中心）为绿色点
        let mut angle_axis = [0.0; 3];
        let mut center = [0.0; 3];
        for i in 0..self.num_cameras() {
            let camera = &parameters[self.camera_block_size() as usize * i as usize..];
            self.camera_to_angle_axis_and_center(camera, &mut angle_axis, &mut center);
            writeln!(file, "{} {} {} 0 255 0", center[0], center[1], center[2])?;
        }

        // 导出结构（即3D点）为白色点
        let points = &parameters[self.camera_block_size() as usize * self.num_cameras as usize..];
        for i in 0..self.num_points() {
            let point = &points[i as usize * self.point_block_size() as usize..];
            for j in 0..self.point_block_size() {
                write!(file, "{} ", point[j as usize])?;
            }
            writeln!(file, "255 255 255")?;
        }

        Ok(())
    }

    /// 将相机参数转换为角轴和中心
    fn camera_to_angle_axis_and_center(&self, camera: &[f64], angle_axis: &mut [f64], center: &mut [f64]) {
        if self.use_quaternions {
            let quaternion: [f64; 4] = [camera[0], camera[1], camera[2], camera[3]];
            let mut angle_axis_arr: [f64; 3] = [0.0; 3];
            quaternion_to_angle_axis(&quaternion, &mut angle_axis_arr);
            angle_axis.copy_from_slice(&angle_axis_arr);
        } else {
            angle_axis.copy_from_slice(&camera[0..3]);
        }

        // c = -R't
        let inverse_rotation = DVector::from_column_slice(angle_axis).map(|x| -x);
        // 修正 angle_axis_rotate_point 的调用
        let angle_axis_arr: [f64; 3] = [angle_axis[0], angle_axis[1], angle_axis[2]];
        let pt_arr: [f64; 3] = [
            camera[self.camera_block_size() as usize - 6],
            camera[self.camera_block_size() as usize - 5],
            camera[self.camera_block_size() as usize - 4],
        ];
        let mut result_arr: [f64; 3] = [0.0; 3];
        let angle_axis_arr: [f64; 3] = [angle_axis[0], angle_axis[1], angle_axis[2]];
        let pt_arr: [f64; 3] = [pt_arr[0], pt_arr[1], pt_arr[2]];
        let mut result_arr: [f64; 3] = [0.0; 3];
        angle_axis_rotate_point(&angle_axis_arr, &pt_arr, &mut result_arr);
        center.copy_from_slice(&result_arr);
    }

    /// 将角轴和中心转换为相机参数
    fn angle_axis_and_center_to_camera(&self, angle_axis: &[f64], center: &[f64], camera: &mut [f64]) {
        if self.use_quaternions {
            let angle_axis_arr: [f64; 3] = [angle_axis[0], angle_axis[1], angle_axis[2]];
            let mut quaternion_arr: [f64; 4] = [0.0; 4];
            angle_axis_to_quaternion(&angle_axis_arr, &mut quaternion_arr);
            camera[0..4].copy_from_slice(&quaternion_arr);
        } else {
            camera[0..3].copy_from_slice(angle_axis);
        }

        // t = -R * c
        let angle_axis_arr: [f64; 3] = [angle_axis[0], angle_axis[1], angle_axis[2]];
        let center_arr: [f64; 3] = [center[0], center[1], center[2]];
        let mut result_arr: [f64; 3] = [0.0; 3];
        angle_axis_rotate_point(&angle_axis_arr, &center_arr, &mut result_arr);
        camera[self.camera_block_size() as usize - 6..self.camera_block_size() as usize - 3].copy_from_slice(&result_arr);
    }

    /// 归一化
    pub fn normalize(&mut self) {
        // 提前获取 num_points、num_cameras 和 camera_block_size
        let num_points = self.num_points;
        let num_cameras = self.num_cameras;
        let camera_block_size = self.camera_block_size() as usize;
    
        // 计算几何的边际中值
        let mut tmp = vec![0.0; num_points as usize];
        let mut median_point = Vector3::zeros();
        let mut parameters = self.parameters.borrow_mut(); // 获取可变引用
        let points = &mut parameters[self.camera_block_size() as usize * self.num_cameras as usize..];
    
        for i in 0..3 {
            for j in 0..num_points {
                tmp[j as usize] = points[3 * j as usize + i];
            }
            median_point[i] = median(&tmp);
        }
    
        for i in 0..num_points {
            let point = DVector::from_column_slice(&points[3 * i as usize..3 * i as usize + 3]);
            tmp[i as usize] = (point - median_point).norm();
        }
    
        let median_absolute_deviation = median(&tmp);
    
        // 缩放使得结果的绝对中位差为100
        let scale = 100.0 / median_absolute_deviation;
    
        // X = scale * (X - median)
        for i in 0..num_points {
            let point = DVector::from_column_slice(&points[3 * i as usize..3 * i as usize + 3]);
            let scaled_point = scale * (point - median_point);
            points[3 * i as usize..3 * i as usize + 3].copy_from_slice(scaled_point.as_slice());
        }

        let cameras = &mut parameters[..self.camera_block_size() as usize * self.num_cameras as usize];

        let mut angle_axis = [0.0; 3];
        let mut center = [0.0; 3];
        for i in 0..num_cameras {
            let camera = &mut cameras[camera_block_size * (i as usize)..];
            self.camera_to_angle_axis_and_center(camera, &mut angle_axis, &mut center);
            // center = scale * (center - median)
            let scaled_center = scale * (DVector::from_column_slice(&center) - median_point);
            center.copy_from_slice(scaled_center.as_slice());
            self.angle_axis_and_center_to_camera(&angle_axis, &center, camera);
        }
    }

    /// 扰动
    pub fn perturb(&mut self, rotation_sigma: f64, translation_sigma: f64, point_sigma: f64) {
        assert!(point_sigma >= 0.0);
        assert!(rotation_sigma >= 0.0);
        assert!(translation_sigma >= 0.0);
    
        // 提前复制参数
        let num_points = self.num_points;
        let camera_block_size = self.camera_block_size() as usize;
        let mut parameters = self.parameters.borrow_mut(); // 获取可变引用
        let points = &mut parameters[self.camera_block_size() as usize * self.num_cameras as usize..];
        let num_cameras = self.num_cameras();

        if point_sigma > 0.0 {
            for i in 0..num_points {
                perturb_point3(point_sigma, &mut points[3 * i as usize..3 * i as usize + 3]);
            }
        }
    
        for i in 0..num_cameras {
            let camera = &mut parameters[camera_block_size * (i as usize)..];
            let mut angle_axis = [0.0; 3];
            let mut center = [0.0; 3];
            self.camera_to_angle_axis_and_center(camera, &mut angle_axis, &mut center);
    
            if rotation_sigma > 0.0 {
                perturb_point3(rotation_sigma, &mut angle_axis);
            }
            self.angle_axis_and_center_to_camera(&angle_axis, &center, camera);
    
            if translation_sigma > 0.0 {
                perturb_point3(translation_sigma, &mut camera[camera_block_size - 6..]);
            }
        }
    }

    /// 获取相机块大小
    pub fn camera_block_size(&self) -> i32 {
        if self.use_quaternions { 10 } else { 9 }
    }

    /// 获取点块大小
    pub fn point_block_size(&self) -> i32 {
        3
    }

    /// 获取相机数量
    pub fn num_cameras(&self) -> i32 {
        self.num_cameras
    }

    /// 获取点数量
    pub fn num_points(&self) -> i32 {
        self.num_points
    }

    /// 获取观测数量
    pub fn num_observations(&self) -> i32 {
        self.num_observations
    }

    /// 获取参数数量
    pub fn num_parameters(&self) -> i32 {
        self.num_parameters
    }

    /// 获取点索引
    pub fn point_index(&self) -> &[i32] {
        &self.point_index
    }

    /// 获取相机索引
    pub fn camera_index(&self) -> &[i32] {
        &self.camera_index
    }

    /// 获取观测数据
    pub fn observations(&self) -> &[f64] {
        &self.observations
    }

    /// 获取参数数据
    pub fn parameters(&self) -> std::cell::Ref<Vec<f64>> {
        self.parameters.borrow() // 返回 Ref<Vec<f64>>
    }

    /// 获取相机参数
    pub fn cameras(&self) -> std::cell::Ref<Vec<f64>> {
        self.parameters.borrow() // 返回 Ref<Vec<f64>>
    }

    /// 获取点参数
    pub fn points(&self) -> Ref<[f64]> {
        let start = self.camera_block_size() as usize * self.num_cameras as usize;
        std::cell::Ref::map(self.parameters.borrow(), |params| &params[start..])
    }

    /// 获取可变的相机参数
    pub fn mutable_cameras(&mut self) -> std::cell::RefMut<Vec<f64>> {
        self.parameters.borrow_mut() // 返回 RefMut<Vec<f64>>
    }

    /// 获取可变的点参数
    pub fn mutable_points(&mut self) -> RefMut<[f64]> {
        let start = self.camera_block_size() as usize * self.num_cameras as usize;
        std::cell::RefMut::map(self.parameters.borrow_mut(), |params| &mut params[start..])
    }

    /// 获取可变的相机参数（针对特定观测）
    pub fn mutable_camera_for_observation(&mut self, i: usize) -> RefMut<[f64]> {
        let start = self.camera_index[i] as usize * self.camera_block_size() as usize;
        let camera_block_size = self.camera_block_size() as usize;
        std::cell::RefMut::map(self.parameters.borrow_mut(), |params| &mut params[start..start + camera_block_size])
    }

    /// 获取可变的点参数（针对特定观测）
    pub fn mutable_point_for_observation(&mut self, i: usize) -> RefMut<[f64]> {
        let start = self.camera_block_size() as usize * self.num_cameras as usize + self.point_index[i] as usize * self.point_block_size() as usize;
        let point_block_size = self.point_block_size() as usize;
        std::cell::RefMut::map(self.parameters.borrow_mut(), |params| &mut params[start..start + point_block_size])
    }

    /// 获取相机参数（针对特定观测）
    pub fn camera_for_observation(&self, i: usize) -> Ref<[f64]> {
        let start = self.camera_index[i] as usize * self.camera_block_size() as usize;
        std::cell::Ref::map(self.parameters.borrow(), |params| &params[start..start + self.camera_block_size() as usize])
    }

    /// 获取点参数（针对特定观测）
    pub fn point_for_observation(&self, i: usize) -> Ref<[f64]> {
        let start = self.camera_block_size() as usize * self.num_cameras as usize + self.point_index[i] as usize * self.point_block_size() as usize;
        std::cell::Ref::map(self.parameters.borrow(), |params| &params[start..start + self.point_block_size() as usize])
    }

    /// 设置相机参数
    pub fn set_cameras(&mut self, cameras: &[f64]) {
        let mut parameters = self.parameters.borrow_mut();
        let camera_block_size = self.camera_block_size() as usize;
        let num_cameras = self.num_cameras as usize;

        // 确保传入的 cameras 数据长度正确
        // assert_eq!(
        //     cameras.len(),
        //     camera_block_size * num_cameras,
        //     "cameras length mismatch"
        // );

        // 计算需要复制的长度
        let copy_len = std::cmp::min(camera_block_size * num_cameras, cameras.len());

        // 将 cameras 数据复制到 parameters 中, 防止越界
        parameters[..copy_len].copy_from_slice(&cameras[..copy_len]);

    } // end fn set_cameras

    /// 设置点参数
    pub fn set_points(&mut self, points: &[f64]) {
        let mut parameters = self.parameters.borrow_mut();
        let camera_block_size = self.camera_block_size() as usize;
        let num_cameras = self.num_cameras as usize;
        let point_block_size = self.point_block_size() as usize;
        let num_points = self.num_points as usize;

        // 确保传入的 points 数据长度正确
        // assert_eq!(
        //     points.len(),
        //     point_block_size * num_points,
        //     "points length mismatch"
        // );

        // 计算需要复制的长度
        let start = camera_block_size * num_cameras;
        let copy_len = std::cmp::min(point_block_size * num_points, points.len());

        // 将 points 数据复制到 parameters 中, 只复制有效的数据部分
        parameters[start..start + copy_len].copy_from_slice(&points[..copy_len]);
        
    } // end fn set_points

}

/// 扰动3D点
fn perturb_point3(sigma: f64, point: &mut [f64]) {
    for i in 0..3 {
        point[i] += rand_normal() * sigma;
    }
}

/// 计算中位数
fn median(data: &[f64]) -> f64 {
    let mut data = data.to_vec();
    data.sort_by(|a, b| a.partial_cmp(b).unwrap());
    data[data.len() / 2]
}

/* end BAL问题 */