#![allow(unused_imports)]

//! 本节演示了如何根据assets/ch11-data/目录下的十张图训练字典

use std::path::Path;
use bye_abow_rs::{keypoint::load_img_get_kps, vocabulary::Vocabulary};

fn main() {
    // 读取图像
    println!("读取图像...");
    let mut images = Vec::new();
    for i in 0..10 {
        let path = format!("./assets/ch11-data/{}.png", i + 1);
        images.push(path);
    }

    // 检测 ORB 特征
    println!("检测 ORB 特征...");
    let n_keypoints = 100;
    let mut descriptors = Vec::new();
    for image_path in &images {
        if let Ok(descriptor) = load_img_get_kps(Path::new(image_path), n_keypoints) {
            descriptors.push(descriptor);
        }
    }

    // 创建词汇表
    println!("创建词汇表...");
    // 将 descriptors 转换为二维数组的切片
    let descriptors_flat: Vec<[u8; 32]> = descriptors.into_iter().flatten().collect();
    let vocab = Vocabulary::create(&descriptors_flat, 9, 3);
    println!("词汇表信息: {:#?}", vocab);

    // 保存词汇表(DBoW3格式)
    vocab.save_dbow3("vocabulary.yml").unwrap();
    println!("完成");
}