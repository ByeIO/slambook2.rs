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

use image::{
    open, ImageBuffer, Rgb, DynamicImage
};
use imageproc::{
    drawing::draw_cross_mut, drawing::draw_line_segment_mut
};

use bye_orb_rs::{
    orb, fast, common::Matchable
};

use std::time::Instant;

fn main() {
    let img1_path = "./assets/ch7-1.png";
    let img2_path = "./assets/ch7-2.png";

    main_orb(img1_path, img2_path);
}

fn main_orb(img1_path: &str, img2_path: &str) {
    // 读取图像
    let mut img1 = open(img1_path).unwrap();
    let mut img2 = open(img2_path).unwrap();

    // 设置关键点数量
    let n_keypoints = 500;

    // 第一步: 检测Oriented FAST角点位置并计算BRIEF描述子
    let start_time = Instant::now();
    let img1_keypoints = orb::orb(&mut img1, n_keypoints).unwrap();
    let img2_keypoints = orb::orb(&mut img2, n_keypoints).unwrap();
    let end_time = Instant::now();
    println!("提取ORB特征点耗时: {:?} 秒", end_time - start_time);

    // 第二步: 使用Hamming距离进行匹配
    let start_time = Instant::now();
    let pair_indices = orb::match_brief(&img1_keypoints, &img2_keypoints);
    let end_time = Instant::now();
    println!("匹配ORB特征点耗时: {:?} 秒", end_time - start_time);

    // 第三步: 匹配点对筛选
    let mut matches: Vec<(usize, usize, f32)> = pair_indices.iter()
        .map(|&(i, j)| (i, j, img1_keypoints[i].distance(&img2_keypoints[j]) as f32))
        .collect();

    matches.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

    let min_dist = matches[0].2;
    let max_dist = matches[matches.len() - 1].2;

    println!("-- 最大距离: {}", max_dist);
    println!("-- 最小距离: {}", min_dist);

    // 当描述子之间的距离大于两倍的最小距离时，认为匹配有误。但最小距离可能会非常小，设置一个经验值30作为下限。
    let good_matches: Vec<(usize, usize, f32)> = matches.iter()
        .filter(|&&(_, _, dist)| dist <= (2.0 * min_dist).max(30.0))
        .cloned()
        .collect();

    // 第四步: 绘制匹配结果
    let mut img1_rgb = img1.to_rgb8();
    let mut img2_rgb = img2.to_rgb8();

    // 绘制所有匹配点
    for &(i, j, _) in &matches {
        let kp1 = (img1_keypoints[i].x, img1_keypoints[i].y);
        let kp2 = (img2_keypoints[j].x, img2_keypoints[j].y);
        let color = Rgb([0, 255, 0]);
        draw_cross_mut(&mut img1_rgb, color, kp1.0 as i32, kp1.1 as i32);
        draw_cross_mut(&mut img2_rgb, color, kp2.0 as i32, kp2.1 as i32);
    }

    // 保存绘制结果
    img1_rgb.save("all_matches.png").unwrap();
    img2_rgb.save("all_matches2.png").unwrap();

    // 绘制筛选后的匹配点
    let mut img1_rgb_good = img1.to_rgb8();
    let mut img2_rgb_good = img2.to_rgb8();

    for &(i, j, _) in &good_matches {
        let kp1 = (img1_keypoints[i].x, img1_keypoints[i].y);
        let kp2 = (img2_keypoints[j].x, img2_keypoints[j].y);
        let color = Rgb([0, 255, 0]);
        draw_cross_mut(&mut img1_rgb_good, color, kp1.0 as i32, kp1.1 as i32);
        draw_cross_mut(&mut img2_rgb_good, color, kp2.0 as i32, kp2.1 as i32);
    }

    // 保存绘制结果
    img1_rgb_good.save("good_matches.png").unwrap();
    img2_rgb_good.save("good_matches2.png").unwrap();

    // 第五步: 创建并排放置的图像
    let (width1, height1) = img1_rgb.dimensions();
    let (width2, height2) = img2_rgb.dimensions();
    let total_width = width1 + width2;
    let max_height = height1.max(height2);

    let mut combined_image = ImageBuffer::new(total_width, max_height);

    // 将第一张图片复制到组合图像的左侧
    for y in 0..height1 {
        for x in 0..width1 {
            combined_image.put_pixel(x, y, *img1_rgb.get_pixel(x, y));
        }
    }

    // 将第二张图片复制到组合图像的右侧
    for y in 0..height2 {
        for x in 0..width2 {
            combined_image.put_pixel(width1 + x, y, *img2_rgb.get_pixel(x, y));
        }
    }

    // 绘制连接线
    for &(i, j, _) in &good_matches {
        let kp1 = (img1_keypoints[i].x, img1_keypoints[i].y);
        let kp2 = (img2_keypoints[j].x, img2_keypoints[j].y);
        let color = Rgb([0, 255, 0]);
        draw_line_segment_mut(
            &mut combined_image,
            (kp1.0 as f32, kp1.1 as f32),
            ((width1 as f32 + kp2.0 as f32), kp2.1 as f32),
            color,
        );
    }

    // 保存并排放置的图像
    combined_image.save("combined_matches.png").unwrap();
}