#![allow(unused_imports)]

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
    let train_dir = "assets/train";
    let test_dir = "assets/test";
    let voc_path = "result/test2.voc";

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
    let voc = Vocabulary::load(voc_path).unwrap();
    println!("词汇表: {:#?}", voc);

    // 从测试数据创建 BoW 向量
    println!("处理测试图片...");
    let mut bows: Vec<(PathBuf, BoW)> = Vec::new();
    let entries: Vec<_> = Path::new(test_dir).read_dir().expect("错误").flatten().collect();
    let total_entries = entries.len();

    for (i, entry) in entries.iter().enumerate() {
        if entry.path().extension().and_then(OsStr::to_str) == Some("png") {    
            // 只处理.png文件
            let mut new_feat = None;
            for k in (1..=n_keypoints).rev() {
                if let Ok(feat) = load_img_get_kps(&entry.path(), k) {
                    new_feat = Some(feat);
                    break;
                }
            }
            if let Some(feat) = new_feat {
                println!("\n{:#?} 特征数量: {}", entry.path().file_name().unwrap(), feat.len());
                bows.push((
                    entry.path(),
                    voc.transform_with_direct_idx(&feat).unwrap().0,
                ));
            }
        }
        print_progress((i + 1) as f32 / total_entries as f32);
    }

    // 使用 L1 范数将几张图片与整个集合进行比较
    println!("\n比较图片...");
    for (f1, bow1) in bows.iter().take(5) {
        let mut scores: Vec<(f32, &OsStr)> = Vec::new();
        for (f2, bow2) in bows.iter() {
            let d = bow1.l1(bow2);
            scores.push((d, f2.file_name().unwrap()));
        }

        // 打印出每张图片的前 5 个匹配项
        println!("\n{:#?} 的前 5 个匹配项:", f1.file_name().unwrap());
        println!("匹配项      |      分数");
        scores.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        for m in scores[..5].iter() {
            println!("{:#?} | {:#?}", m.1, m.0);
        }
    }
}