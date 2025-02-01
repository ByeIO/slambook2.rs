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

use std::fs::File;
use std::io::{
    self, BufReader, Read, Write, 
};
use std::path::Path;

use image::{
    DynamicImage, ImageBuffer, Luma
};
use nalgebra::{
    Isometry3, Quaternion, Vector3, 
    Translation, Translation3, Unit,
};

use bye_octomap_rs::{
    PointCloud::Pointcloud
};
use bye_octomap_rs::OkTree as _OkTree;
use _OkTree::OkTree;

fn main() -> io::Result<()> {
    // 彩色图和深度图
    let mut color_imgs: Vec<DynamicImage> = Vec::new();
    let mut depth_imgs: Vec<ImageBuffer<Luma<u16>, Vec<u16>>> = Vec::new();
    // 相机位姿
    let mut poses: Vec<Isometry3<f64>> = Vec::new();

    // 读取相机位姿文件
    let mut fin = BufReader::new(File::open("./assets/ch12-data/pose.txt")?);
    for i in 0..5 {
        // 读取彩色图和深度图
        let color_path = format!("./assets/ch12-data/color/{}.png", i + 1);
        let depth_path = format!("./assets/ch12-data/depth/{}.png", i + 1);
        color_imgs.push(image::open(color_path).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?);
        depth_imgs.push(image::open(depth_path).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?.into_luma16());

        // 读取位姿数据
        let mut data = [0.0; 7];
        for j in 0..7 {
            let mut buffer = [0u8; 8]; // 8字节缓冲区用于读取f64
            fin.read_exact(&mut buffer)?;
            data[j] = f64::from_le_bytes(buffer); // 将字节转换为f64
        }
        let q = Quaternion::new(data[6], data[3], data[4], data[5]);
        let t = Translation3::new(data[0], data[1], data[2]);
        let pose = Isometry3::from_parts(t, Unit::new_normalize(q));
        poses.push(pose);
    }

    // 相机内参
    let cx = 319.5;
    let cy = 239.5;
    let fx = 481.2;
    let fy = -480.0;
    let depth_scale = 5000.0;

    println!("正在将图像转换为 Octomap ...");

    // 初始化八叉树, 参数为分辨率
    let mut tree = OkTree::new(0.01); 

    for i in 0..5 {
        println!("转换图像中: {}", i + 1);
        let color = &color_imgs[i];
        let depth = &depth_imgs[i];
        let t = poses[i];

        // 八叉树中的点云
        let mut cloud = Pointcloud::new();  

        for v in 0..color.height() {
            for u in 0..color.width() {
                // 深度值
                let d = depth.get_pixel(u, v)[0]; 
                // 为0表示没有测量到
                if d == 0 { continue; } 
                let mut point = Vector3::zeros();
                point[2] = f64::from(d) / depth_scale;
                point[0] = (f64::from(u) - cx) * point[2] / fx;
                point[1] = (f64::from(v) - cy) * point[2] / fy;
                let point_world = t * point;
                // 将世界坐标系的点放入点云
                cloud.push_back(point_world[0], point_world[1], point_world[2]);
            }
        }

        // 将点云存入八叉树地图，给定原点，这样可以计算投射线
        let origin = Vector3::new(t.translation.x, t.translation.y, t.translation.z);
        tree.insert_pointcloud(cloud, origin);
    }

    // 更新中间节点的占据信息并写入磁盘
    tree.update_inner_occupancy();
    println!("saving octomap ... ");
    let mut file = File::create("./result/ch12-dense_RGBD-octomap_mapping-octomap.bt").unwrap();
    tree.write_binary(&mut file)?;
    Ok(())
}