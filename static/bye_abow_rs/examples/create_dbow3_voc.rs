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
    // 加载现有词汇表
    let vocab = Vocabulary::load("result/test2.voc").unwrap();
    println!("词汇表: {:#?}", vocab);

    // 保存为 DBoW3 格式的 YAML 文件并压缩为 .gz 文件
    let file_path_gz = "./result/test_vocab_dbow3.yml.gz";
    assert!(vocab.save_dbow3(file_path_gz).is_ok());

    // 从 DBoW3 格式的 YAML 文件加载词典
    let loaded_vocab_gz = Vocabulary::load_dbow3(file_path_gz);
    assert!(loaded_vocab_gz.is_ok());

    // 保存为 DBoW3 格式的 YAML 文件
    let file_path_yml = "./result/test_vocab_dbow3.yml";
    assert!(vocab.save_dbow3(file_path_yml).is_ok());

    // 从 DBoW3 格式的 YAML 文件加载词典
    let loaded_vocab_yml = Vocabulary::load_dbow3(file_path_yml);
    assert!(loaded_vocab_yml.is_ok());
}