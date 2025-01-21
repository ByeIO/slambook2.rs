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

use image::{
    GenericImageView, ImageBuffer, Rgba, RgbaImage, 
    DynamicImage, ImageFormat, imageops::FilterType
};

use std::time::Instant;
use rand::Rng;

fn main() {
    // 畸变参数
    let k1 = -0.28340811;
    let k2 = 0.07395907;
    let p1 = 0.00019359;
    let p2 = 1.76187114e-05;

    // 内参
    let fx = 458.654;
    let fy = 457.296;
    let cx = 367.215;
    let cy = 248.375;

    // 读取图片
    let image_path = "./assets/ch5-distorted.png";
    let original_image = image::open(image_path).expect("failed to open image file");
    
    let (rows, cols) = (original_image.height(), original_image.width());
    let mut image_undistort = ImageBuffer::new(cols as u32, rows as u32);

    // 计算去畸变后图像的内容
    for v in 0..rows {
        for u in 0..cols {
            // 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted, v_distorted)
            let x = (u as f64 - cx) / fx;
            let y = (v as f64 - cy) / fy;
            let r = (x * x + y * y).sqrt();
            let x_distorted = x * (1.0 + k1 * r * r + k2 * r * r * r * r) + 2.0 * p1 * x * y + p2 * (r * r + 2.0 * x * x);
            let y_distorted = y * (1.0 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2.0 * y * y) + 2.0 * p2 * x * y;
            let u_distorted = fx * x_distorted + cx;
            let v_distorted = fy * y_distorted + cy;
            // 赋值 (最近邻插值)
            if u_distorted >= 0.0 && v_distorted >= 0.0 && u_distorted < cols as f64 && v_distorted < rows as f64 {
                let pixel = original_image.get_pixel(u_distorted as u32, v_distorted as u32);
                image_undistort.put_pixel(u as u32, v as u32, pixel);
            } else {
                image_undistort.put_pixel(u as u32, v as u32, Rgba([0, 0, 0, 0]));
            }
        } // end for u
    } // end for v

    // 保存去畸变后的图像
    image_undistort.save("./undistorted.png").unwrap();
}