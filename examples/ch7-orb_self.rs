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
    open, ImageBuffer, Rgb, DynamicImage,
    Luma
};
use imageproc::{
    drawing::draw_cross_mut, drawing::draw_line_segment_mut
};

use bye_orb_rs::{
    orb, fast, common::Matchable
};

use std::time::Instant;

use nalgebra::Point2;

// 全局变量
const FIRST_FILE: &str = "./assets/ch7-1.png";
const SECOND_FILE: &str = "./assets/ch7-2.png";

// 描述符类型
type DescType = Vec<u32>;

/// 计算ORB描述符
fn compute_orb(img: &ImageBuffer<Luma<u8>, Vec<u8>>, keypoints: &Vec<Point2<f32>>) -> Vec<DescType> {
    let half_patch_size = 8;
    let half_boundary = 16;
    let mut descriptors = Vec::new();
    let mut bad_points = 0;

    for kp in keypoints {
        // 超出边界
        if kp.x < half_boundary as f32 || kp.y < half_boundary as f32 ||
           kp.x >= img.width() as f32 - half_boundary as f32 || kp.y >= img.height() as f32 - half_boundary as f32 {
            bad_points += 1;
            descriptors.push(Vec::new());
            continue;
        }

        let mut m01 = 0.0;
        let mut m10 = 0.0;

        for dx in -half_patch_size..half_patch_size {
            for dy in -half_patch_size..half_patch_size {
                let pixel = img.get_pixel((kp.x + dx as f32) as u32, (kp.y + dy as f32) as u32)[0];
                m10 += dx as f32 * pixel as f32;
                m01 += dy as f32 * pixel as f32;
            }
        }

        let m_sqrt = (m01 * m01 + m10 * m10).sqrt() + 1e-18;
        let sin_theta = m01 / m_sqrt;
        let cos_theta = m10 / m_sqrt;

        let mut desc = vec![0; 8];
        for i in 0..8 {
            let mut d = 0;
            for k in 0..32 {
                let idx_pq = i * 32 + k;

                // 从元组中获取数据
                let p = Point2::new(ORB_PATTERN[idx_pq].0 as f32, ORB_PATTERN[idx_pq].1 as f32);
                let q = Point2::new(ORB_PATTERN[idx_pq].2 as f32, ORB_PATTERN[idx_pq].3 as f32);

                // 旋转角度
                let pp = Point2::new(
                    cos_theta * p.x - sin_theta * p.y + kp.x,
                    sin_theta * p.x + cos_theta * p.y + kp.y,
                );
                let qq = Point2::new(
                    cos_theta * q.x - sin_theta * q.y + kp.x,
                    sin_theta * q.x + cos_theta * q.y + kp.y,
                );

                /* start 像素比较 */
                if pp.x >= 0.0 && pp.x < img.width() as f32 && pp.y >= 0.0 && pp.y < img.height() as f32 &&
                qq.x >= 0.0 && qq.x < img.width() as f32 && qq.y >= 0.0 && qq.y < img.height() as f32 {
                    // 确保坐标在图像范围内
                    let px = pp.x as u32;
                    let py = pp.y as u32;
                    let qx = qq.x as u32;
                    let qy = qq.y as u32;

                    if px < img.width() && py < img.height() && qx < img.width() && qy < img.height() {
                            if img.get_pixel(px, py)[0] < img.get_pixel(qx, qy)[0] {
                            if k < img.width() as usize && k < img.height() as usize {
                                d |= 1 << k;
                            }
                        }
                    } else {
                        println!("Warning: Pixel index out of bounds: pp({}, {}), qq({}, {})", px, py, qx, qy);
                    }
                } else {
                    // 处理越界情况，例如跳过该点或使用默认值
                    println!("Warning: Pixel index out of bounds: pp({}, {}), qq({}, {})", pp.x, pp.y, qq.x, qq.y);
                }
                /* end 像素比较 */

            }// end for k
            desc[i] = d;
        }
        descriptors.push(desc);
    }

    println!("bad/total: {}/{}", bad_points, keypoints.len());
    descriptors
}

/// 暴力匹配描述符
fn bf_match(desc1: &Vec<DescType>, desc2: &Vec<DescType>) -> Vec<(usize, usize, u32)> {
    let d_max = 40;
    let mut matches = Vec::new();

    for (i1, d1) in desc1.iter().enumerate() {
        if d1.is_empty() {
            continue;
        }
        let mut best_match = (0, 256);
        for (i2, d2) in desc2.iter().enumerate() {
            if d2.is_empty() {
                continue;
            }
            let mut distance = 0;
            for k in 0..8 {
                distance += (d1[k] ^ d2[k]).count_ones();
            }
            if distance < d_max && distance < best_match.1 {
                best_match = (i2, distance);
            }
        }
        if best_match.1 < d_max {
            matches.push((i1, best_match.0, best_match.1));
        }
    }

    matches
}

fn main() {
    // 读取图像
    let mut first_image: DynamicImage = open(FIRST_FILE).unwrap();
    let mut second_image: DynamicImage = open(SECOND_FILE).unwrap();

    // 将图像转换为灰度图像
    let first_image_gray = first_image.clone().to_luma8();
    let second_image_gray = second_image.clone().to_luma8();

    // 设置关键点数量
    let n_keypoints = 500;

    // 检测FAST关键点
    let start_time = Instant::now();
    let keypoints1 = orb::orb(&mut first_image, n_keypoints).unwrap();
    let keypoints2 = orb::orb(&mut second_image, n_keypoints).unwrap();

    // 将 Brief 转换为 Point2<f32>
    let keypoints1: Vec<Point2<f32>> = keypoints1.iter().map(|kp| Point2::new(kp.x as f32, kp.y as f32)).collect();
    let keypoints2: Vec<Point2<f32>> = keypoints2.iter().map(|kp| Point2::new(kp.x as f32, kp.y as f32)).collect();

    // 计算描述符
    let descriptor1 = compute_orb(&first_image_gray, &keypoints1);
    let descriptor2 = compute_orb(&second_image_gray, &keypoints2);

    let end_time = Instant::now();
    println!("提取ORB耗时: {:?} 秒", end_time - start_time);

    // 匹配描述符
    let start_time = Instant::now();
    let matches = bf_match(&descriptor1, &descriptor2);
    let end_time = Instant::now();
    println!("匹配ORB耗时: {:?} 秒", end_time - start_time);
    println!("匹配点数量: {}", matches.len());

    // 将图像转换为RGB格式
    let mut first_image_rgb = first_image.to_rgb8();
    let mut second_image_rgb = second_image.to_rgb8();

    // 获取图像的尺寸
    let (width1, height1) = first_image_rgb.dimensions();
    let (width2, height2) = second_image_rgb.dimensions();

    // 计算组合图像的尺寸
    let total_width = width1 + width2;
    let max_height = height1.max(height2);

    // 创建一个新的图像缓冲区来存储组合图像
    let mut combined_image = ImageBuffer::new(total_width, max_height);

    // 将第一张图片复制到组合图像的左侧
    for y in 0..height1 {
        for x in 0..width1 {
            combined_image.put_pixel(x, y, *first_image_rgb.get_pixel(x, y));
        }
    }

    // 将第二张图片复制到组合图像的右侧
    for y in 0..height2 {
        for x in 0..width2 {
            combined_image.put_pixel(width1 + x, y, *second_image_rgb.get_pixel(x, y));
        }
    }

    // 绘制连接线
    for &(i1, i2, _) in &matches {
        let kp1 = keypoints1[i1];
        let kp2 = keypoints2[i2];
        let color = Rgb([0, 255, 0]); // 绿色
        draw_line_segment_mut(
            &mut combined_image,
            (kp1.x as f32, kp1.y as f32),
            ((width1 as f32 + kp2.x as f32), kp2.y as f32),
            color,
        );
    }

    // 保存组合图像
    combined_image.save("ch7-orb_self-combined_matches.png").unwrap();
    println!("完成.");
}

// ORB模式
const ORB_PATTERN: [(i32, i32, i32, i32); 256] = [
    (8, -3, 9, 5), (4, 2, 7, -12), (-11, 9, -8, 2), (7, -12, 12, -13),
    (2, -13, 2, 12), (1, -7, 1, 6), (-2, -10, -2, -4), (-13, -13, -11, -8),
    (-13, -3, -12, -9), (10, 4, 11, 9), (-13, -8, -8, -9), (-11, 7, -9, 12),
    (7, 7, 12, 6), (-4, -5, -3, 0), (-13, 2, -12, -3), (-9, 0, -7, 5),
    (12, -6, 12, -1), (-3, 6, -2, 12), (-6, -13, -4, -8), (11, -13, 12, -8),
    (4, 7, 5, 1), (5, -3, 10, -3), (3, -7, 6, 12), (-8, -7, -6, -2),
    (-2, 11, -1, -10), (-13, 12, -8, 10), (-7, 3, -5, -3), (-4, 2, -3, 7),
    (-10, -12, -6, 11), (5, -12, 6, -7), (5, -6, 7, -1), (1, 0, 4, -5),
    (9, 11, 11, -13), (4, 7, 4, 12), (2, -1, 4, 4), (-4, -12, -2, 7),
    (-8, -5, -7, -10), (4, 11, 9, 12), (0, -8, 1, -13), (-13, -2, -8, 2),
    (-3, -2, -2, 3), (-6, 9, -4, -9), (8, 12, 10, 7), (0, 9, 1, 3),
    (7, -5, 11, -10), (-13, -6, -11, 0), (10, 7, 12, 1), (-6, -3, -6, 12),
    (10, -9, 12, -4), (-13, 8, -8, -12), (-13, 0, -8, -4), (3, 3, 7, 8),
    (5, 7, 10, -7), (-1, 7, 1, -12), (3, -10, 5, 6), (2, -4, 3, -10),
    (-13, 0, -13, 5), (-13, -7, -12, 12), (-13, 3, -11, 8), (-7, 12, -4, 7),
    (6, -10, 12, 8), (-9, -1, -7, -6), (-2, -5, 0, 12), (-12, 5, -7, 5),
    (3, -10, 8, -13), (-7, -7, -4, 5), (-3, -2, -1, -7), (2, 9, 5, -11),
    (-11, -13, -5, -13), (-1, 6, 0, -1), (5, -3, 5, 2), (-4, -13, -4, 12),
    (-9, -6, -9, 6), (-12, -10, -8, -4), (10, 2, 12, -3), (7, 12, 12, 12),
    (-7, -13, -6, 5), (-4, 9, -3, 4), (7, -1, 12, 2), (-7, 6, -5, 1),
    (-13, 11, -12, 5), (-3, 7, -2, -6), (7, -8, 12, -7), (-13, -7, -11, -12),
    (1, -3, 12, 12), (2, -6, 3, 0), (-4, 3, -2, -13), (-1, -13, 1, 9),
    (7, 1, 8, -6), (1, -1, 3, 12), (9, 1, 12, 6), (-1, -9, -1, 3),
    (-13, -13, -10, 5), (7, 7, 10, 12), (12, -5, 12, 9), (6, 3, 7, 11),
    (5, -13, 6, 10), (2, -12, 2, 3), (3, 8, 4, -6), (2, 6, 12, -13),
    (9, -12, 10, 3), (-8, 4, -7, 9), (-11, 12, -4, -6), (1, 12, 2, -8),
    (6, -9, 7, -4), (2, 3, 3, -2), (6, 3, 11, 0), (3, -3, 8, -8),
    (7, 8, 9, 3), (-11, -5, -6, -4), (-10, 11, -5, 10), (-5, -8, -3, 12),
    (-10, 5, -9, 0), (8, -1, 12, -6), (4, -6, 6, -11), (-10, 12, -8, 7),
    (4, -2, 6, 7), (-2, 0, -2, 12), (-5, -8, -5, 2), (7, -6, 10, 12),
    (-9, -13, -8, -8), (-5, -13, -5, -2), (8, -8, 9, -13), (-9, -11, -9, 0),
    (1, -8, 1, -2), (7, -4, 9, 1), (-2, 1, -1, -4), (11, -6, 12, -11),
    (-12, -9, -6, 4), (3, 7, 7, 12), (5, 5, 10, 8), (0, -4, 2, 8),
    (-9, 12, -5, -13), (0, 7, 2, 12), (-1, 2, 1, 7), (5, 11, 7, -9),
    (3, 5, 6, -8), (-13, -4, -8, 9), (-5, 9, -3, -3), (-4, -7, -3, -12),
    (6, 5, 8, 0), (-7, 6, -6, 12), (-13, 6, -5, -2), (1, -10, 3, 10),
    (4, 1, 8, -4), (-2, -2, 2, -13), (2, -12, 12, 12), (-2, -13, 0, -6),
    (4, 1, 9, 3), (-6, -10, -3, -5), (-3, -13, -1, 1), (7, 5, 12, -11),
    (4, -2, 5, -7), (-13, 9, -9, -5), (7, 1, 8, 6), (7, -8, 7, 6),
    (-7, -4, -7, 1), (-8, 11, -7, -8), (-13, 6, -12, -8), (2, 4, 3, 9),
    (10, -5, 12, 3), (-6, -5, -6, 7), (8, -3, 9, -8), (2, -12, 2, 8),
    (-11, -2, -10, 3), (-12, -13, -7, -9), (-11, 0, -10, -5), (5, -3, 11, 8),
    (-2, -13, -1, 12), (-1, -8, 0, 9), (-13, -11, -12, -5), (-10, -2, -10, 11),
    (-3, 9, -2, -13), (2, -3, 3, 2), (-9, -13, -4, 0), (-4, 6, -3, -10),
    (-4, 12, -2, -7), (-6, -11, -4, 9), (6, -3, 6, 11), (-13, 11, -5, 5),
    (11, 11, 12, 6), (7, -5, 12, -2), (-1, 12, 0, 7), (-4, -8, -3, -2),
    (-7, 1, -6, 7), (-13, -12, -8, -13), (-7, -2, -6, -8), (-8, 5, -6, -9),
    (-5, -1, -4, 5), (-13, 7, -8, 10), (1, 5, 5, -13), (1, 0, 10, -13),
    (9, 12, 10, -1), (5, -8, 10, -9), (-1, 11, 1, -13), (-9, -3, -6, 2),
    (-1, -10, 1, 12), (-13, 1, -8, -10), (8, -11, 10, -6), (2, -13, 3, -6),
    (7, -13, 12, -9), (-10, -10, -5, -7), (-10, -8, -8, -13), (4, -6, 8, 5),
    (3, 12, 8, -13), (-4, 2, -3, -3), (5, -13, 10, -12), (4, -13, 5, -1),
    (-9, 9, -4, 3), (0, 3, 3, -9), (-12, 1, -6, 1), (3, 2, 4, -8),
    (-10, -10, -10, 9), (8, -13, 12, 12), (-8, -12, -6, -5), (2, 2, 3, 7),
    (10, 6, 11, -8), (6, 8, 8, -12), (-7, 10, -6, 5), (-3, -9, -3, 9),
    (-1, -13, -1, 5), (-3, -7, -3, 4), (-8, -2, -8, 3), (4, 2, 12, 12),
    (2, -5, 3, 11), (6, -9, 11, -13), (3, -1, 7, 12), (11, -1, 12, 4),
    (-3, 0, -3, 6), (4, -11, 4, 12), (2, -4, 2, 1), (-10, -6, -8, 1),
    (-13, 7, -11, 1), (-13, 12, -11, -13), (6, 0, 11, -13), (0, -1, 1, 4),
    (-13, 3, -9, -2), (-9, 8, -6, -3), (-13, -6, -8, -2), (5, -9, 8, 10),
    (2, 7, 3, -9), (-1, -6, -1, -1), (9, 5, 11, -2), (11, -3, 12, -8),
    (3, 0, 3, 5), (-1, 4, 0, 10), (3, -6, 4, 5), (-13, 0, -10, 5),
    (5, 8, 12, 11), (8, 9, 9, -6), (7, -4, 8, -12), (-10, 4, -10, 9),
    (7, 3, 12, 4), (9, -7, 10, -2), (7, 0, 12, -2), (-1, -6, 0, -11),
];