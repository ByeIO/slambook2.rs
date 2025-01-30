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

use std::cmp::{min, max};
use image::{ImageError, GenericImageView, DynamicImage, ImageBuffer, GrayImage, ImageFormat, RgbImage};
use image::imageops::{blur};
use cgmath::{prelude::{*},Rad, Deg};
use bitvector::BitVector;

use crate::{fast, brief, common};
use fast::{FastKeypoint};
use common::{*};

// Consts
const DEFAULT_BRIEF_LENGTH:usize = 256;

//
// Sobel Calculations
//

type SobelFilter = [[i32;3];3];
const SOBEL_X : SobelFilter = [[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]];
const SOBEL_Y : SobelFilter = [[1, 2, 1], [0, 0, 0], [-1, -2, -1]];
const SOBEL_OFFSETS : [[(i32, i32);3];3] = [[(-1, -1), (0, -1), (1, -1)], [(-1, 0),  (0, 0), (1, 0)], [(-1, 1), (0, 1), (1, 1)]];

/// 使用Sobel算子计算图像的梯度
/// # Safety
/// 该函数使用了不安全的`unsafe_get_pixel`方法，调用者需要确保传入的坐标在图像范围内
unsafe fn sobel(img: &image::GrayImage, filter: &SobelFilter, x: i32, y: i32) -> u8 {
    let mut sobel:i32 = 0;
    for (i, row) in filter.iter().enumerate() {
        for (j, k) in row.iter().enumerate() {
            if *k == 0 { continue }

            let offset = SOBEL_OFFSETS[i][j];
            let (x, y) = ((x + offset.0) as u32, (y + offset.1) as u32);
            let px = img.unsafe_get_pixel(x, y).0[0];
            sobel += px as i32 * *k;
        }
    }
    min(sobel.abs() as u8, u8::MAX)
}

/// 创建Sobel梯度图像
fn create_sobel_image(img: &GrayImage) -> GrayImage {
    let mut new_image:GrayImage = ImageBuffer::new(img.width(), img.height());

    for y in 1..img.height()-1 {
        for x in 1..img.width()-1 {
            let mut px = new_image.get_pixel_mut(x, y);
            px.0[0] = unsafe { sobel(img, &SOBEL_Y, x as i32, y as i32) as u8 };
        }
    }

    new_image
}

//
// BRIEF Calculations
//

#[derive(Debug)]
pub struct Brief {
    pub x: i32,
    pub y: i32,
    pub b: BitVector
}

impl Matchable for Brief {
    fn distance(&self, other: &Self) -> usize {
        (0..min(self.b.capacity(), other.b.capacity()))
        .fold(0, |acc, x| {
            acc + (self.b.contains(x) != other.b.contains(x)) as usize
        })
    }
}

/// 将角度四舍五入到最近的增量
fn round_angle(angle: i32, increment: i32) -> i32 {
    let modulo:i32 = angle % increment;
    let complement:i32 = if angle < 0 
        { increment + modulo } else { increment - modulo} ;

    if modulo.abs() > (increment << 1) {
        return if angle < 0 { angle - complement } else { angle + complement };
    }

    angle - modulo
}

/// 计算BRIEF描述子
pub fn brief(blurred_img: &GrayImage, vec: &Vec<FastKeypoint>, brief_length: Option<usize>) -> Vec<Brief> {
    let brief_length = brief_length.unwrap_or(DEFAULT_BRIEF_LENGTH);
    let width:i32 = blurred_img.width() as i32;
    let height:i32 = blurred_img.height() as i32;

    // copy offsets into current frame on stack
    let offsets = brief::OFFSETS.clone();

    vec.into_iter()
        .map(|k| {
            let rotation = Deg::from(Rad(k.moment.rotation)).0.round() as i32;
            let rounded_angle = Deg(round_angle(rotation, 12) as f32);

            let cos_a = Deg::cos(rounded_angle);
            let sin_a = Deg::sin(rounded_angle);
            let (x, y) = k.location;

            let mut bit_vec = BitVector::new(brief_length);

            for (i, ((x0, y0), (x1, y1))) in offsets.iter().enumerate() {
                let mut steered_p1 = (
                    x + (x0 * cos_a - y0 * sin_a).round() as i32,
                    y + (x0 * sin_a + y0 * cos_a).round() as i32
                );

                let mut steered_p2 = (
                    x + (x1 * cos_a - y1 * sin_a).round() as i32,
                    y + (x1 * sin_a + y1 * cos_a).round() as i32
                );

                steered_p1.0 = max(min(steered_p1.0, width - 1), 0);
                steered_p2.0 = max(min(steered_p2.0, width - 1), 0);
                steered_p1.1 = max(min(steered_p1.1, height - 1), 0);
                steered_p2.1 = max(min(steered_p2.1, height - 1), 0);

                let brief_feature = blurred_img.get_pixel(steered_p1.0 as u32, steered_p1.1 as u32).0[0] >
                                    blurred_img.get_pixel(steered_p2.0 as u32, steered_p2.1 as u32).0[0];

                if brief_feature {
                    bit_vec.insert(i);
                }
            }

            Brief {
                x: k.location.0,
                y: k.location.1,
                b: bit_vec
            } 
        })
        .collect::<Vec<Brief>>()
}

//
// ORB Calculations
//

/// 计算ORB特征点和描述子
pub fn orb(img: &DynamicImage, n:usize) -> Result<Vec<Brief>, ImageError> {
    let gray_img = img.to_luma8();
    
    let mut keypoints:Vec<FastKeypoint> = fast::fast(&gray_img, None, None)?;

    let keypoints = adaptive_nonmax_suppression(&mut keypoints, n);

    let blurred_img = blur(&gray_img, 3.0);
    let brief_descriptors = brief(&blurred_img, &keypoints, None);

    Ok(brief_descriptors)
}

/// 匹配两幅图像的关键点
pub fn match_brief(img1_keypoints: &Vec<Brief>, img2_keypoints: &Vec<Brief>) -> Vec<(usize, usize)> {
    let mut pair_indices = Vec::new();

    // 遍历第一幅图像的所有关键点
    for (i, kp1) in img1_keypoints.iter().enumerate() {
        let mut best_match_index = 0;
        let mut best_distance = usize::MAX;

        // 遍历第二幅图像的所有关键点，找到与当前关键点最匹配的点
        for (j, kp2) in img2_keypoints.iter().enumerate() {
            let distance = kp1.distance(kp2);

            // 如果找到更小的距离，则更新最佳匹配
            if distance < best_distance {
                best_distance = distance;
                best_match_index = j;
            }
        }

        // 如果找到的匹配距离小于某个阈值，则认为是一个有效的匹配
        if best_distance < 50 { // 这里的阈值可以根据实际情况调整
            pair_indices.push((i, best_match_index));
        }
    }

    pair_indices
}