#![allow(unused_imports)]

use bye_abow_rs::{
    keypoint::{
        all_kps_from_dir, 
        load_img_get_kps
    }, 
    vocabulary::Vocabulary, 
};

fn main() {

    // 从png图像中提取 orb 描述符(仅支持png图片)
    let n_keypoints: usize = 10;
    let features = all_kps_from_dir("assets/train", n_keypoints).unwrap();
    println!("检测到 {} 个 ORB 特征。", features.len());

    // 从特征创建词汇表
    let voc = Vocabulary::create(&features, 9, 3);
    println!("\n词汇表 = {:#?}", voc);

    // 保存词汇表，然后再次加载它，仅为了示范用法
    voc.save("result/test.voc").unwrap();
    let loaded_voc = Vocabulary::load("result/test.voc").unwrap();

    // 确保保存和加载操作成功
    assert_eq!(voc, loaded_voc);
}