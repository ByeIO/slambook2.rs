#![allow(dead_code)] // 允许未使用的代码
#![allow(unused_variables)] // 允许未使用的变量
#![allow(unused_imports)] // 允许未使用的导入
#![allow(unused_mut)] // 允许未使用的可变变量
#![allow(unused_assignments)] // 允许未使用的赋值
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // 在非调试模式下设置Windows子系统
#![allow(rustdoc::missing_crate_level_docs)] // 允许缺少crate级别的文档
#![allow(unsafe_code)] // 允许使用unsafe代码
#![allow(clippy::undocumented_unsafe_blocks)] // 允许未文档化的unsafe块
#![allow(unused_must_use)] // 允许未使用的must_use结果
#![allow(non_snake_case)] // 允许非蛇形命名

use image::{
    open, ImageFormat, Luma, ImageBuffer, Rgb,
    DynamicImage
};

use orbrs::{
    orb, fast
};

fn main() {
    test1();
    test2();
    println!("Hello, world!");
}

fn test1() {
    // 打开第一张图像
    let mut img1 = image::open("./assets/money1.jpg").unwrap();
    // 打开第二张图像
    let mut img2 = image::open("./assets/money2.jpg").unwrap();

    // 设置关键点数量
    let n_keypoints = 50;

    // 计算第一张图像的关键点
    let img1_keypoints = orbrs::orb::orb(&mut img1, n_keypoints).unwrap();
    // 计算第二张图像的关键点
    let img2_keypoints = orbrs::orb::orb(&mut img2, n_keypoints).unwrap();

    // 匹配两张图像的关键点
    let pair_indices = orbrs::orb::match_brief(&img1_keypoints, &img2_keypoints);

    // 打印匹配的关键点对
    println!("pair_indices:{:?}", pair_indices);
}
fn test2() {
    let mut img = image::open("./assets/money3.jpg").unwrap();
    let mut gray_img: ImageBuffer<Luma<u8>, Vec<u8>> = img.to_luma8();

    let fast_keypoints = orbrs::fast::fast(&gray_img, Some(orbrs::fast::FastType::TYPE_9_16), None).unwrap();

    // 在图像上绘制关键点
    // 创建一个具有相同尺寸的RGB图像
    let mut rgb_img = ImageBuffer::new(gray_img.width(), gray_img.height());

    // 将RGB图像转换为DynamicImage
    let dynamic_img = DynamicImage::ImageRgb8(rgb_img.clone());

    // 将DynamicImage转换为RGBA图像
    let mut rgba_img = dynamic_img.to_rgba8();

    // 将灰度值复制到RGB图像的所有三个通道中
    for (x, y, pixel) in gray_img.enumerate_pixels() {
        let gray_value = pixel[0];
        rgb_img.put_pixel(x, y, Rgb([gray_value, gray_value, gray_value]));
    }
    
    orbrs::fast::draw_moments(&mut rgba_img, &fast_keypoints);
    img.save_with_format("fast_output.png", image::ImageFormat::Png);
}