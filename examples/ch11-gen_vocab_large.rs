#![allow(unused_imports)]

//! 本节演示了如何根据assets/ch11-rgbd_dataset_freiburg1_xyz目录下的图像训练大型字典

use std::path::Path;
use std::fs::File;
use std::io::{BufRead, BufReader};
use bye_abow_rs::{keypoint::load_img_get_kps, vocabulary::Vocabulary};

fn main() {
    // 读取数据集路径
    let dataset_dir = "./assets/ch11-rgbd_dataset_freiburg1_xyz";
    
    // 打开关联文件
    let associate_file = format!("{}/associate.txt", dataset_dir);
    let file = File::open(&associate_file).expect("无法打开associate.txt文件");
    let reader = BufReader::new(file);

    // 存储RGB和深度图像路径
    let mut rgb_files = Vec::new();
    let mut depth_files = Vec::new();

    // 解析关联文件
    for line in reader.lines() {
        if let Ok(line) = line {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() == 4 {
                let rgb_path = format!("{}/{}", dataset_dir, parts[1]);
                let depth_path = format!("{}/{}", dataset_dir, parts[3]);
                
                // 检查文件是否存在
                if Path::new(&rgb_path).exists() && Path::new(&depth_path).exists() {
                    rgb_files.push(rgb_path);
                    depth_files.push(depth_path);
                }
            }
        }
    }

    // 检测 ORB 特征
    println!("检测 ORB 特征...");
    let n_keypoints = 100;
    let mut descriptors = Vec::new();

    // 合并处理存在的RGB和depth图像
    for (rgb_file, depth_file) in rgb_files.iter().zip(&depth_files) {
        if Path::new(rgb_file).exists() {
            if let Ok(descriptor) = load_img_get_kps(Path::new(rgb_file), n_keypoints) {
                descriptors.push(descriptor);
            }
        }
        if Path::new(depth_file).exists() {
            if let Ok(descriptor) = load_img_get_kps(Path::new(depth_file), n_keypoints) {
                descriptors.push(descriptor);
            }
        }
    }

    // 创建词汇表
    println!("创建词汇表...");
    let descriptors_flat: Vec<[u8; 32]> = descriptors.into_iter().flatten().collect();
    let vocab = Vocabulary::create(&descriptors_flat, 9, 3);
    println!("词汇表信息: {:#?}", vocab);

    // 保存词汇表(DBoW3格式)
    vocab.save_dbow3("./result/ch11-gen_vocab_large.yml").unwrap();
    println!("完成");
}