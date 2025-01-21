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

// 这个是纯rust的cv库,与opencv无关
use cv::*;

// 图像处理库
use image::{
    GenericImageView, ImageBuffer, Rgba, RgbaImage,
    DynamicImage, ImageFormat, imageops::FilterType
};

// 随机数
use rand::Rng;

use std::time::Instant;

fn main() {
    // 随机数生成器
    let mut rng = rand::thread_rng();

    // 1. 读取图片,打印图片基本信息
    // 加载图片
    let image_path = "./assets/ch5-ubuntu.png";
    let original_image = image::open(image_path).expect("failed to open image file");
    
    // 获取图片的格式（扩展名）
    // let format = original_image.format();
    // let format_str = match format {
    //     Some(fmt) => fmt.to_string(),
    //     None => "未知格式".to_string(),
    // };
    // 获取图片的颜色类型
    let color_type = original_image.color();
    let color_type_str = match color_type {
        image::ColorType::Rgb8 => "RGB8",
        image::ColorType::Rgba8 => "RGBA8",
        image::ColorType::L8 => "L8",
        image::ColorType::La8 => "LA8",
        image::ColorType::Rgb16 => "RGB16",
        image::ColorType::Rgba16 => "RGBA16",
        _ => "未知颜色类型",
    };
    println!("图片长:{}, 图片宽:{}, 图片扩展名:{}, 图片颜色空间:{}.",
        original_image.height(),
        original_image.width(),
        image_path,
        color_type_str
    );

    // 2. 遍历图片像素并打印用时
    let start_time = Instant::now();
    let pixels = original_image.as_rgba8().unwrap();
    for pixel in pixels.pixels() {
        // 这里可以处理每个像素，现在只是遍历
    }
    let duration = start_time.elapsed();
    println!("遍历图片像素用时: {:.2?}", duration);

    // 3. 直接在原图上将左上角100*100的块置白色并另存为到图片
    let mut modified_image = original_image.clone();
    if let Some(image_buffer) = modified_image.as_mut_rgba8() {
        for x in 0..100 {
            for y in 0..100 {
                image_buffer.put_pixel(x, y, Rgba([255, 255, 255, 255]));
            }
        }
    }
    modified_image.save("./modified_white.png").unwrap();

    // 4. 拷贝一份图像,将左上角100*100的块置黑色并另存为到图片
    let mut copied_image = original_image.clone();
    if let Some(image_buffer) = copied_image.as_mut_rgba8() {
        for x in 0..100 {
            for y in 0..100 {
                image_buffer.put_pixel(x, y, Rgba([0, 0, 0, 255]));
            }
        }
    }
    copied_image.save("./modified_black.png").unwrap();
}