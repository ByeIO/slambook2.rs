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

//! 点云建图

use image::{open, DynamicImage};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use nalgebra::{
    Isometry3, Quaternion, Vector3, 
    Rotation3, Translation3, SMatrix, Unit,
    Matrix3, Matrix4,
};

use bye_pcl_rs::common::{PointXYZRGB, PointCloud};
use bye_pcl_rs::filters::{voxel_grid::*, statistical_outlier_removal::*};
use bye_pcl_rs::f3l_filter::F3lFilter;

// 自定义一个包装类型，实现所需的 trait
#[derive(Clone, Copy, Debug)]
struct WrappedPointXYZRGB(pub PointXYZRGB);

impl std::ops::Index<usize> for WrappedPointXYZRGB {
    type Output = f32;

    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.0.x,
            1 => &self.0.y,
            2 => &self.0.z,
            _ => panic!("Index out of bounds"),
        }
    }
}

impl From<WrappedPointXYZRGB> for [f32; 3] {
    fn from(point: WrappedPointXYZRGB) -> [f32; 3] {
        [point.0.x, point.0.y, point.0.z]
    }
}

impl From<[f32; 3]> for WrappedPointXYZRGB {
    fn from(arr: [f32; 3]) -> Self {
        let mut point = PointXYZRGB::default();
        point.x = arr[0];
        point.y = arr[1];
        point.z = arr[2];
        WrappedPointXYZRGB(point)
    }
}

fn main() {
    // 彩色图和深度图的向量
    let mut color_imgs: Vec<DynamicImage> = Vec::new();
    let mut depth_imgs: Vec<DynamicImage> = Vec::new();
    // 相机位姿的向量
    let mut poses: Vec<Isometry3<f64>> = Vec::new();

    // 打开位姿文件
    let file = match File::open("./assets/ch12-data/pose.txt") {
        Ok(file) => file,
        Err(_) => {
            eprintln!("cannot find pose file");
            return;
        }
    };
    let reader = BufReader::new(file);
    let mut lines = reader.lines();

    for i in 0..5 {
        // 图像文件格式
        let color_path = format!("./assets/ch12-data/color/{}.png", i + 1);
        let depth_path = format!("./assets/ch12-data/depth/{}.png", i + 1);

        // 读取彩色图像
        let color_img = match open(&color_path) {
            Ok(img) => img,
            Err(_) => {
                eprintln!("Failed to read color image: {}", color_path);
                continue;
            }
        };
        color_imgs.push(color_img);

        // 读取深度图像
        let depth_img = match open(&depth_path) {
            Ok(img) => img,
            Err(_) => {
                eprintln!("Failed to read depth image: {}", depth_path);
                continue;
            }
        };
        depth_imgs.push(depth_img);

        // 读取位姿数据
        let mut data = [0.0; 7];
        for j in 0..7 {
            if let Some(Ok(line)) = lines.next() {
                if let Ok(num) = line.parse::<f64>() {
                    data[j] = num;
                } else {
                    eprintln!("Failed to parse pose data");
                    return;
                }
            } else {
                eprintln!("Not enough pose data");
                return;
            }
        }

        // 从四元数创建旋转
        let q = Quaternion::new(data[6], data[3], data[4], data[5]);
        // 将 Quaternion 转换为 UnitQuaternion
        let unit_q = Unit::new_normalize(q);
        // 将 UnitQuaternion 转换为旋转矩阵
        let rotation_matrix = unit_q.to_rotation_matrix();
        // 创建平移
        let translation = Translation3::from(Vector3::new(data[0], data[1], data[2]));
        // 组合旋转和平移得到等距变换
        let T = Isometry3::from_parts(translation, rotation_matrix.into());
        poses.push(T);
    }

    // 相机内参
    let cx = 319.5;
    let cy = 239.5;
    let fx = 481.2;
    let fy = -480.0;
    let depthScale = 5000.0;

    println!("正在将图像转换为点云...");
    
    // 新建一个点云
    let mut point_cloud = PointCloud::<PointXYZRGB>::new();

    for i in 0..5 {
        println!("转换图像中: {}", i + 1);
        let color = color_imgs[i].to_rgb8();
        let depth = depth_imgs[i].to_luma16();
        let T = poses[i];

        let mut current = PointCloud::<PointXYZRGB>::new();

        for v in 0..color.height() {
            for u in 0..color.width() {
                let d = depth.get_pixel(u, v)[0];
                if d == 0 {
                    continue;
                }

                let mut point = Vector3::new(0.0, 0.0, 0.0);
                point[2] = d as f64 / depthScale;
                point[0] = (u as f64 - cx) * point[2] / fx;
                point[1] = (v as f64 - cy) * point[2] / fy;

                let pointWorld = T * point;

                let mut p = PointXYZRGB::default();
                p.x = pointWorld[0] as f32;
                p.y = pointWorld[1] as f32;
                p.z = pointWorld[2] as f32;

                let pixel = color.get_pixel(u, v);
                p.rgb = ((pixel[0] as u32) << 16) | ((pixel[1] as u32) << 8) | (pixel[2] as u32);

                current.points.push(p);
            }
        }

        // 深度及统计异常值移除滤波器
        let mut sor = StatisticalOutlierRemoval::new(1.0, 50);
        let wrapped_current_points: Vec<WrappedPointXYZRGB> = current.points.iter().map(|p| WrappedPointXYZRGB(p.clone())).collect();
        let filtered_wrapped_points = sor.filter_instance(&wrapped_current_points);
        let filtered_points: Vec<PointXYZRGB> = filtered_wrapped_points.into_iter().map(|wp| wp.0).collect();
        let mut tmp = PointCloud::<PointXYZRGB>::new();
        tmp.points = filtered_points;

        point_cloud.points.extend(tmp.points);
    }

    point_cloud.is_dense = false;
    println!("点云共有 {} 个点.", point_cloud.points.len());
    
    // 体素网格滤波器
    let leaf_size = [0.01, 0.01, 0.01];
    let mut voxel_grid = VoxelGrid::new();
    let wrapped_point_cloud_points: Vec<WrappedPointXYZRGB> = point_cloud.points.iter().map(|p| WrappedPointXYZRGB(p.clone())).collect();
    let voxel_filtered_wrapped_points = voxel_grid.filter_instance(&wrapped_point_cloud_points);
    let voxel_filtered_points: Vec<PointXYZRGB> = voxel_filtered_wrapped_points.into_iter().map(|wp| wp.0).collect();
    let mut voxel_filtered_cloud = PointCloud::<PointXYZRGB>::new();
    voxel_filtered_cloud.points = voxel_filtered_points;
    println!("体素滤波后点云共有 {} 个点.", voxel_filtered_cloud.points.len());

    // 保存文件
    let file_result = File::create("./result/ch12-dense_RGBD-pointcloud_mapping_output.ply");
    match file_result {
        Ok(file) => {
            let mut writer = BufWriter::new(file);

            // 写入文件头
            if let Err(e) = writeln!(writer, "ply") {
                eprintln!("写入文件失败: {}", e);
                return;
            }
            if let Err(e) = writeln!(writer, "format ascii 1.0") {
                eprintln!("写入文件失败: {}", e);
                return;
            }
            if let Err(e) = writeln!(writer, "element vertex {}", voxel_filtered_cloud.points.len()) {
                eprintln!("写入文件失败: {}", e);
                return;
            }
            if let Err(e) = writeln!(writer, "property float x") {
                eprintln!("写入文件失败: {}", e);
                return;
            }
            if let Err(e) = writeln!(writer, "property float y") {
                eprintln!("写入文件失败: {}", e);
                return;
            }
            if let Err(e) = writeln!(writer, "property float z") {
                eprintln!("写入文件失败: {}", e);
                return;
            }
            if let Err(e) = writeln!(writer, "property uchar red") {
                eprintln!("写入文件失败: {}", e);
                return;
            }
            if let Err(e) = writeln!(writer, "property uchar green") {
                eprintln!("写入文件失败: {}", e);
                return;
            }
            if let Err(e) = writeln!(writer, "property uchar blue") {
                eprintln!("写入文件失败: {}", e);
                return;
            }
            if let Err(e) = writeln!(writer, "end_header") {
                eprintln!("写入文件失败: {}", e);
                return;
            }

            // 写入点云数据
            for point in voxel_filtered_cloud.points {
                let r = ((point.rgb >> 16) & 0xFF) as u8;
                let g = ((point.rgb >> 8) & 0xFF) as u8;
                let b = (point.rgb & 0xFF) as u8;
                if let Err(e) = writeln!(
                    writer,
                    "{:.6} {:.6} {:.6} {} {} {}",
                    point.x, point.y, point.z, r, g, b
                ) {
                    eprintln!("写入文件失败: {}", e);
                    return;
                }
            }

            println!("点云已保存为 output.ply");
        }
        Err(e) => {
            eprintln!("无法创建文件: {}", e);
        }
    }
    
}
