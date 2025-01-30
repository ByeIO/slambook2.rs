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
#![allow(clippy::upper_case_acronyms)]

use image::{open, ImageBuffer, Rgb, DynamicImage};
use imageproc::{
    drawing::draw_cross_mut, drawing::draw_line_segment_mut
};

use bye_orb_rs::{
    orb, fast, common::Matchable
};

use crate::{
    BowErr, BowResult, Desc
};

use std::time::Instant;
use std::path::Path;
use std::fs::read_dir;

// 从图像中提取ORB关键点描述符
pub fn orb_from_image(img: &mut DynamicImage, n_keypoints: usize) -> Vec<Desc> {
    let keypoints = orb::orb(img, n_keypoints).unwrap();
    keypoints.into_iter().map(|kp| {
        let mut desc = [0u8; 32];
        desc[0] = kp.x as u8;
        desc[1] = kp.y as u8;
        
        // 通过遍历`BitVector`的每一位，将其转换为`u8`数组。
        // 每个`u8`可以存储8个二进制位，因此我们使用`desc[i / 8] |= 1 << (i % 8)`来设置相应的位。
        for i in 0..32 {
            if i < kp.b.capacity() && kp.b.contains(i) {
                desc[i / 8] |= 1 << (i % 8);
            }
        }
        
        desc
    }).collect()
}

/// 加载图像并提取ORB关键点描述符
pub fn load_img_get_kps<P: AsRef<Path>>(path: P, n_keypoints: usize) -> BowResult<Vec<Desc>> {
    let mut img = open(path.as_ref()).unwrap();
    // 返回值
    Ok(orb_from_image(&mut img, n_keypoints))
}

/// 从目录中的所有图像中提取ORB关键点描述符, 叠加到同一个Vec中
pub fn all_kps_from_dir<P: AsRef<Path>>(path: P, n_keypoints: usize) -> BowResult<Vec<Desc>> {
    let mut features: Vec<Desc> = Vec::new();
    for entry in read_dir(path.as_ref()).unwrap().flatten() {
        let entry_path = entry.path();
        // 仅支持png格式
        if entry_path.extension().and_then(|s| s.to_str()) == Some("png") {
            println!("Extracting keypoint descriptors from {:?}", entry_path);
            // 尝试使用n_keypoints到1的顺序尝试提取ORB描述符,直到成功,
            // 如果都不行则忽略这次。
            for k in (1..=n_keypoints).rev() {
                if let Ok(descs) = load_img_get_kps(&entry_path, k) {
                    if !descs.is_empty() {
                        features.extend(descs);
                        break;
                    }
                }
            }
        }
    }
    // 返回值
    Ok(features)
}
#[cfg(test)]
mod tests2 {
    use super::*;
    use std::path::PathBuf;
    use rand::*;

    #[test]
    fn test_orb_from_image() {
        // 创建一个测试图像
        let img = image::DynamicImage::new_rgb8(100, 100);
        // 随机大小的正方形，边长在10到59之间
        let square_size = rand::random::<u32>() % 50 + 10; 
        let x = (100 - square_size) / 2;
        let y = (100 - square_size) / 2;
        let mut img: ImageBuffer<image::Rgba<u8>, Vec<u8>> = img.into_rgba8();
        // 绘制白色正方形
        img.put_pixel(x, y, image::Rgba([255, 255, 255, 255])); 
        let n_keypoints = 1;
        let descs = orb_from_image(&mut img.into(), n_keypoints);
        assert_eq!(descs.len(), n_keypoints);
    }

    #[test]
    fn test_load_img_get_kps() {
        // 创建一个测试图像, 仅支持png格式
        let path = PathBuf::from("./test_img.png");
        let img = image::DynamicImage::new_rgb8(100, 100);
        // 随机大小的正方形，边长在10到59之间
        let square_size = rand::random::<u32>() % 50 + 10; 
        let x = (100 - square_size) / 2;
        let y = (100 - square_size) / 2;
        let mut img: ImageBuffer<image::Rgba<u8>, Vec<u8>> = img.into_rgba8();
        // 绘制白色正方形
        img.put_pixel(x, y, image::Rgba([255, 255, 255, 255])); 
        img.save(&path).unwrap();

        // 从文件中读取图像并提取ORB关键点描述符
        let n_keypoints = 1;
        let descs = load_img_get_kps(&path, n_keypoints).unwrap();
        println!("descs: {:?}", descs);
        assert!(descs.len() >= n_keypoints);

        // 删除测试图像文件
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn test_all_kps_from_dir() {
        use image::GenericImage;
        // 创建一个测试目录和多个测试图像文件
        let dir_path = PathBuf::from(format!("./test_dir_{}", rand::random::<u32>() % 50 + 10 ));
        std::fs::create_dir_all(&dir_path).unwrap();
        for i in 0..3 {
            // 仅支持png格式
            let path = dir_path.join(format!("{}.png", i));
            println!("path: {:?}", path);
            let mut img = image::DynamicImage::new_rgb8(100, 100);
            let square_size = rand::random::<u32>() % 50 + 10; // 随机大小的正方形，边长在10到59之间
            let x = (100 - square_size) / 2;
            let y = (100 - square_size) / 2;
            img.put_pixel(x, y, image::Rgba([255, 255, 255, 255])); // 绘制白色正方形
            img.save(&path).unwrap();
        }

        let n_keypoints = 1;
        let descs = all_kps_from_dir(&dir_path, n_keypoints).unwrap();
        assert_eq!(descs.len(), 3 * n_keypoints);

        // 删除测试目录和图像文件
        std::fs::remove_dir_all(dir_path).unwrap();
    } // end test_all_kps_from_dir

}
