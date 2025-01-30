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

fn main() {
    // 从 DBoW3 格式的 YAML 文件加载词典
    let file_path = "./assets/ch11-vocabulary-regex.yml";
    // let file_path = "./assets/vocab.yml";
    
    // FIXME: 解析跨越多行的descriptor时会有: Descriptor length mismatch: TryFromSliceError(())
    let loaded_vocab_gz = Vocabulary::load_dbow3(file_path).expect("REASON");

    // 从测试数据创建 BoW 向量, 保存文件以供演示。
    let mut bows: Vec<(PathBuf, BoW)> = Vec::new();
    let n_keypoints: usize = 1;
    
    for entry in Path::new("assets/test").read_dir().expect("错误").flatten() {
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
                println!("{:#?} 特征数量: {}", entry.path().file_name().unwrap(), feat.len());
                bows.push((
                    entry.path(),
                    loaded_vocab_gz.transform_with_direct_idx(&feat).unwrap().0,
                ));
            }
        }
    }

    // 使用 L1 范数将几张图片与整个集合进行比较
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