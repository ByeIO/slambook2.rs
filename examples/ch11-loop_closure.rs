#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(unused_variables)]

use std::{
    ffi::OsStr,
    path::{Path, PathBuf},
};

use bye_abow_rs::{
    keypoint::{
        all_kps_from_dir, 
        load_img_get_kps
    }, 
    vocabulary::Vocabulary, 
    BoW, 
};

fn print_progress(progress: f32) {
    let filled = (progress * 20.0) as usize;
    let empty = 20 - filled;
    print!("\r[");
    for _ in 0..filled {
        print!("#");
    }
    for _ in 0..empty {
        print!(" ");
    }
    print!("] {:.0}%", progress * 100.0);
    std::io::Write::flush(&mut std::io::stdout()).unwrap();
}

fn main() {
    // 从png图像中提取 orb 描述符(仅支持png图片)
    let n_keypoints: usize = 100;
    let train_dir = "assets/ch11-data";
    let test_dir = "assets/ch11-data";
    let voc_path = "result/ch11-loop_closure.voc";

    // 提取训练图片的特征
    println!("提取训练图片的特征...");
    let features = all_kps_from_dir(train_dir, n_keypoints).unwrap();
    println!("检测到 {} 个 ORB 特征。", features.len());

    // 从特征创建词汇表
    println!("创建词汇表...");
    let voc = Vocabulary::create(&features, 9, 3);
    println!("\n词汇表 = {:#?}", voc);

    // 保存词汇表
    println!("保存词汇表...");
    voc.save(voc_path).unwrap();

    // 加载现有词汇表
    println!("加载词汇表...");
    let vocab = Vocabulary::load(voc_path).unwrap();
    if vocab.blocks.is_empty() {
        eprintln!("词汇表不存在。");
        return;
    }
    println!("词汇表: {:#?}", voc);

    // 读取图片
    println!("读取图片...");
    let mut images = Vec::new();
    for i in 0..10 {
        let path = format!("./assets/ch11-data/{}.png", i + 1);
        images.push(Path::new(&path).to_path_buf());
    }

    // 检测 ORB 特征
    println!("检测 ORB 特征...");
    let n_keypoints = 100;
    let mut descriptors = Vec::new();
    for image in &images {
        let mut new_feat = None;
        for k in (1..=n_keypoints).rev() {
            if let Ok(feat) = load_img_get_kps(image, k) {
                new_feat = Some(feat);
                break;
            }
        }
        if let Some(feat) = new_feat {
            descriptors.push(feat);
        }
    }

    // 比较图片与图片
    println!("比较图片与图片...");
    for i in 0..images.len() {
        let bow1 = vocab.transform_with_direct_idx(&descriptors[i]).unwrap().0;
        for j in i..images.len() {
            let bow2 = vocab.transform_with_direct_idx(&descriptors[j]).unwrap().0;
            let score = bow1.l1(&bow2);
            println!("图片 {} vs 图片 {} : {}", i, j, score);
        }
        println!();
    }

    // 比较图片与数据库
    println!("比较图片与数据库...");
    let mut db = Vec::new();
    for descriptor in &descriptors {
        db.push(vocab.transform_with_direct_idx(descriptor).unwrap().0);
    }
    println!("数据库信息: 共 {} 个条目", db.len());
    for i in 0..descriptors.len() {
        let query_bow = vocab.transform_with_direct_idx(&descriptors[i]).unwrap().0;
        let mut scores = Vec::new();
        for (j, db_bow) in db.iter().enumerate() {
            let score = query_bow.l1(db_bow);
            scores.push((score, j));
        }
        scores.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        println!("搜索图片 {} 返回结果: {:?}", i, &scores[..4]);
        println!();
    }

    println!("完成。");
}