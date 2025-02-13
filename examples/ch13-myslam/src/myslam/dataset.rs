#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(unused_variables)]
#![allow(unused_mut)]

//! 数据集处理

// 使用类型别名
use super::preclude::*;

// 标准库
use std::fs::File;
use std::io::{self, BufRead, BufReader, Cursor};
use std::path::Path;
use std::cell::{
    RefMut, Ref, RefCell
};
use std::sync::Arc;
use std::fmt::Write;
use std::time::Instant;

// 日志
use log::{info, error, warn};

// 随机数
use rand::Rng;

// 图像处理
use image::{
    GenericImageView, ImageBuffer, Rgba, RgbaImage, 
    DynamicImage, ImageFormat, imageops::FilterType
};

// 内部库
use super::camera::Camera;
use super::frame::Frame;

/// 数据集读取
/// 构造时传入配置文件路径，配置文件的dataset_dir为数据集路径
/// Init之后可获得相机和下一帧图像
#[derive(Debug, Clone, Default)]
pub struct Dataset {
    // 数据集路径
    pub dataset_path: String,
    // 当前图像索引
    pub current_image_index: usize,
    // 相机列表
    pub cameras: Vec<Arc<Camera>>,
}

impl Dataset {
    /// 构造函数，接收数据集路径作为参数
    pub fn new(dataset_path: &str) -> Self {
        Dataset {
            dataset_path: dataset_path.to_string(),
            current_image_index: 0,
            cameras: Vec::new(),
        }
    }

    /// 初始化，返回是否成功
    pub fn init(&mut self) -> bool {
        // 打开相机校准文件
        let calib_path = Path::new(&self.dataset_path).join("calib.txt");
        let file = match File::open(calib_path) {
            Ok(file) => file,
            Err(_) => {
                error!("cannot find {}/calib.txt!", self.dataset_path);
                return false;
            }
        };
        let reader = BufReader::new(file);
        let mut lines = reader.lines();

        for i in 0..4 {
            // 读取相机名称
            let camera_name = lines.next().unwrap().unwrap();
            // 读取投影数据
            let projection_line = lines.next().unwrap().unwrap();
            let mut projection_data: Vec<f64> = projection_line.split_whitespace()
                .map(|s| s.parse().unwrap())
                .collect();

            // 构建相机内参矩阵 K
            let mut K = Matrix3::zeros();
            K[(0, 0)] = projection_data[0];
            K[(0, 1)] = projection_data[1];
            K[(0, 2)] = projection_data[2];
            K[(1, 0)] = projection_data[4];
            K[(1, 1)] = projection_data[5];
            K[(1, 2)] = projection_data[6];
            K[(2, 0)] = projection_data[8];
            K[(2, 1)] = projection_data[9];
            K[(2, 2)] = projection_data[10];

            // 构建平移向量 t
            let t = Vector3::new(projection_data[3], projection_data[7], projection_data[11]);
            let t = K.try_inverse().unwrap() * t;
            K = K * 0.5;

            // 创建新的相机实例
            let new_camera = Arc::new(Camera::with_params(
                K[(0, 0)], K[(1, 1)], K[(0, 2)], K[(1, 2)],
                t.norm(), SE3::from_rot_trans(SO3::identity(), t),
            ));
            self.cameras.push(new_camera);
            info!("Camera {} extrinsics: {:?}", i, t.transpose());
        }

        self.current_image_index = 0;
        true
    }

    /// 创建并返回包含立体图像的下一帧
    pub fn next_frame(&mut self) -> Option<Arc<Frame>> {
        // 生成图像文件路径
        let mut left_path = String::new();
        let mut right_path = String::new();
        write!(&mut left_path, "{}/image_0/{:06}.png", self.dataset_path, self.current_image_index).unwrap();
        write!(&mut right_path, "{}/image_1/{:06}.png", self.dataset_path, self.current_image_index).unwrap();

        /* start 使用image库转换 */
        
        let current_image_index = 0;
        
        // 读取左右图像
        let image_left = match image::open(left_path) {
            Ok(img) => img.grayscale(),
            Err(_) => {
                warn!("cannot find images at index {}", current_image_index);
                // return;
            }
        };
        let image_right = match image::open(right_path) {
            Ok(img) => img.grayscale(),
            Err(_) => {
                warn!("cannot find images at index {}", current_image_index);
                // return;
            }
        };
    
        // 调整图像大小
        let image_left_resized = image_left.resize_exact(
            (image_left.width() as f64 * 0.5) as u32,
            (image_left.height() as f64 * 0.5) as u32,
            FilterType::Nearest
        );
        let image_right_resized = image_right.resize_exact(
            (image_right.width() as f64 * 0.5) as u32,
            (image_right.height() as f64 * 0.5) as u32,
            FilterType::Nearest
        );
    
        // 将 image 库的图像数据转换为 OMatrix 格式
        let mut image_left_omatrix = OMatrix::from_element(
            image_left_resized.height() as usize,
            image_left_resized.width() as usize,
            0u8
        );
        let mut image_right_omatrix = OMatrix::from_element(
            image_right_resized.height() as usize,
            image_right_resized.width() as usize,
            0u8
        );
    
        for y in 0..image_left_resized.height() {
            for x in 0..image_left_resized.width() {
                let pixel = image_left_resized.get_pixel(x, y)[0];
                image_left_omatrix[(y as usize, x as usize)] = pixel;
            }
        }
    
        for y in 0..image_right_resized.height() {
            for x in 0..image_right_resized.width() {
                let pixel = image_right_resized.get_pixel(x, y)[0];
                image_right_omatrix[(y as usize, x as usize)] = pixel;
            }
        }
    
        // 这里可以使用 image_left_omatrix 和 image_right_omatrix 进行后续操作
        println!("Image processing completed.");
        
        /* end 使用image库转换 */
        
        // 创建新的帧
        let new_frame = Frame::CreateFrame();
        new_frame.left_img_ = image_left_resized;
        new_frame.right_img_ = image_right_resized;
        self.current_image_index += 1;

        Some(new_frame)
    }

    /// 根据相机 ID 获取相机
    pub fn get_camera(&self, camera_id: usize) -> Arc<Camera> {
        self.cameras[camera_id].clone()
    }
}
