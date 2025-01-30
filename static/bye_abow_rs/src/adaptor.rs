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

//! 适配数据到DBoW3格式

// 随机数
use rand::{
    distr::{
        weighted::WeightedIndex, Distribution
    },
    rng, Rng,
};
use rand::prelude::{
    IndexedRandom, IteratorRandom, SliceRandom
};

// 序列化
use serde::{Deserialize, Serialize};
use smallvec::ToSmallVec;
use bitvec::{order::Msb0, view::BitView};
use serde_yml::{to_writer, from_reader};

// 压缩/解压
use flate2::Compression;
use flate2::write::GzEncoder;
use flate2::read::GzDecoder;

// 标准库
use std::fmt;
use std::fs::File;
use std::io::{self, Write, Read};
use std::path::Path;
use std::collections::HashMap;

// 内部库
use crate::{
    BowResult, BoW, Desc, DirectIdx, 
    IdPath, BowErr, 
};
use crate::vocabulary::{
    Vocabulary, NodeId, Block,
    Children, 
};

/* start DBoW3格式适配 */
#[derive(Serialize, Deserialize)]
pub struct DBoW3Node {
    nodeId: u32,
    parentId: u32,
    weight: f32,
    descriptor: String,
}

#[derive(Serialize, Deserialize)]
pub struct DBoW3Word {
    wordId: u32,
    nodeId: u32,
}

#[derive(Serialize, Deserialize)]
pub struct DBoW3Vocabulary {
    k: u32,
    L: u32,
    scoringType: u32,
    weightingType: u32,
    nodes: Vec<DBoW3Node>,
    words: Vec<DBoW3Word>,
}

impl Vocabulary {
    /// 生成 DBoW3 格式的yaml文件及压缩为gz文件
    pub fn save_dbow3<P: AsRef<std::path::Path>>(&self, file: P) -> BowResult<()> {
        let file_path = file.as_ref();
        let mut file = File::create(file_path)?;
    
        if file_path.extension().and_then(|s| s.to_str()) == Some("gz") {
            let mut gz = GzEncoder::new(file, Compression::default());
            Self::generate_dbow3_yaml(self, &mut gz)?;
            gz.finish()?;
        } else {
            Self::generate_dbow3_yaml(self, &mut file)?;
        }
        Ok(())
    }

    /// 从 DBoW3 格式的 YAML 文件加载词典
    pub fn load_dbow3<P: AsRef<std::path::Path>>(file: P) -> BowResult<Self> {
        let file_path = file.as_ref();
        if file_path.extension().and_then(|s| s.to_str()) == Some("gz") {
            let file = File::open(file_path)?;
            let gz = GzDecoder::new(file);
            Self::parse_dbow3_yaml(gz)
        } else {
            let file = File::open(file_path)?;
            Self::parse_dbow3_yaml(file)
        }
    }

    /// 生成 YAML 文件流
    fn generate_dbow3_yaml(vocab: &Vocabulary, writer: &mut impl Write) -> BowResult<()> {
        writeln!(writer, "%YAML 1.0")?;
        writeln!(writer, "vocabulary:")?;
        writeln!(writer, "    k: {}", vocab.k)?;
        writeln!(writer, "    L: {}", vocab.levels)?;
        writeln!(writer, "    scoringType: 0")?;
        writeln!(writer, "    weightingType: 0")?;
        writeln!(writer, "    nodes:")?;
    
        for block in &vocab.blocks {
            for (i, child) in block.children.ids.iter().enumerate() {
                if let NodeId::Block(id) = child {
                    writeln!(writer, "        - {{ nodeId:{}, parentId:{}, weight:{}, descriptor:\"{}\" }}",
                            id, block.id.get_bid(), block.children.weights[i], 
                            block.children.features[i].iter().map(|b| b.to_string()).collect::<Vec<_>>().join(" "))?;
                }
            }
        }
    
        writeln!(writer, "    words:")?;
        for (word_id, node_id) in vocab.blocks.iter().flat_map(|b| b.children.ids.iter().enumerate())
            .filter(|(_, n)| matches!(n, NodeId::Leaf(_)))
            .map(|(i, n)| (i, n.get_bid())) {
            writeln!(writer, "        - {{ wordId:{}, nodeId:{} }}", word_id, node_id)?;
        }
    
        Ok(())
    }

    // 解析 YAML 文件
    fn parse_dbow3_yaml(mut reader: impl Read) -> BowResult<Vocabulary> {
        let mut contents = String::new();
        reader.read_to_string(&mut contents)?;
    
        let mut vocab = Vocabulary::empty(10, 5);
        let lines: Vec<&str> = contents.lines().collect();
    
        let mut current_section = None;
        let mut nodes = Vec::new();
        let mut words = Vec::new();
        let mut i = 0;
    
        println!("开始解析YAML文件，总行数: {}", lines.len());
    
        while i < lines.len() {
            let line = lines[i].trim();
            if line.is_empty() {
                i += 1;
                continue;
            }
    
            if line.starts_with("nodes:") {
                current_section = Some("nodes");
                println!("进入节点解析部分");
            } else if line.starts_with("words:") {
                current_section = Some("words");
                println!("进入词解析部分");
            } else if line.starts_with("- {") {
                let content = line.split('{').nth(1).and_then(|s| s.split('}').next())
                    .unwrap_or("");
                println!("解析内容: {{{}}}", content); // 打印{}包围的内容
    
                match current_section {
                    Some("nodes") => {
                        println!("解析节点信息，当前行: {}", i);
                        let mut node_content = String::new();
                        node_content.push_str(line.split('{').nth(1).unwrap_or(""));
                        i += 1;
    
                        while i < lines.len() && !lines[i].trim().ends_with('}') {
                            node_content.push_str(lines[i].trim());
                            i += 1;
                        }
                        if i < lines.len() {
                            node_content.push_str(lines[i].trim().split('}').next().unwrap_or(""));
                        }
    
                        let mut node_data = HashMap::new();
                        for pair in node_content.split(',') {
                            let kv: Vec<&str> = pair.split(':').collect();
                            if kv.len() == 2 {
                                node_data.insert(kv[0].trim(), kv[1].trim());
                            }
                        }
    
                        let node_id: usize = node_data.get("nodeId")
                            .ok_or(BowErr::ParseError("Missing nodeId".to_string()))?
                            .parse()?;
    
                        let parent_id: usize = node_data.get("parentId")
                            .ok_or(BowErr::ParseError("Missing parentId".to_string()))?
                            .parse()?;
    
                        let weight = node_data.get("weight")
                            .map(|w| w.parse().unwrap_or(1.0))
                            .unwrap_or(1.0);
    
                        let descriptor: Vec<u8> = node_data.get("descriptor")
                            .ok_or(BowErr::ParseError("Missing descriptor".to_string()))?
                            .trim_matches('"')
                            .split_whitespace()
                            .take(32)  // Only take the first 32 elements
                            .map(|s| s.parse().map_err(|_| BowErr::ParseError("Invalid descriptor format".to_string())))
                            .collect::<Result<Vec<_>, _>>()?;

                        // Pad with zeros if there are fewer than 32 elements
                        let mut descriptor = descriptor;
                        descriptor.resize(32, 0);
    
                        println!("成功解析节点: nodeId={}, parentId={}, weight={}, descriptor长度={}", node_id, parent_id, weight, descriptor.len());
                        nodes.push((node_id, parent_id, weight, descriptor));
                    }
                    Some("words") => {
                        println!("解析词信息，当前行: {}", i);
                        let word_content = line.split('{').nth(1).and_then(|s| s.split('}').next())
                            .ok_or(BowErr::ParseError("Invalid word format".to_string()))?;
    
                        let mut word_data = HashMap::new();
                        for pair in word_content.split(',') {
                            let kv: Vec<&str> = pair.split(':').collect();
                            if kv.len() == 2 {
                                word_data.insert(kv[0].trim(), kv[1].trim());
                            }
                        }
    
                        let word_id: usize = word_data.get("wordId")
                            .ok_or(BowErr::ParseError("Missing wordId".to_string()))?
                            .parse()?;
    
                        let node_id: usize = word_data.get("nodeId")
                            .ok_or(BowErr::ParseError("Missing nodeId".to_string()))?
                            .parse()?;
    
                        println!("成功解析词: wordId={}, nodeId={}", word_id, node_id);
                        words.push((word_id, node_id));
                    }
                    _ => {}
                }
            } else if line.starts_with("k:") {
                vocab.k = line.split(':').nth(1).unwrap().trim().parse().unwrap();
                println!("解析k值: {}", vocab.k);
            } else if line.starts_with("L:") {
                vocab.levels = line.split(':').nth(1).unwrap().trim().parse().unwrap();
                println!("解析L值: {}", vocab.levels);
            }
            i += 1;
        }
    
        println!("开始构建词汇表结构，节点数: {}, 词数: {}", nodes.len(), words.len());
        for (node_id, parent_id, weight, descriptor) in &nodes {
            let block = Block {
                id: NodeId::Block(*parent_id),
                children: Children {
                    features: vec![descriptor.as_slice().try_into().expect("Descriptor length mismatch")],
                    weights: vec![*weight],
                    cluster_size: vec![1],
                    ids: vec![NodeId::Block(*node_id)],
                },
            };
            vocab.blocks.push(block);
        }
    
        vocab.num_leaves = words.len();
        vocab.num_blocks = nodes.len();
    
        println!("词汇表解析完成，总词数: {}, 总节点数: {}", vocab.num_leaves, vocab.num_blocks);
        Ok(vocab)
    }

}
/* end DBoW3格式适配 */

#[cfg(test)]
mod tests2 {
    use super::*;

    #[test]
    fn test_save_and_load_dbow3() {
        use crate::keypoint::load_img_get_kps;
        use image::ImageBuffer;
        use std::path::PathBuf;
        // 创建一个测试图像, 仅支持png格式
        // let path = PathBuf::from(format!("./test_img_{}.png", rand::random::<u32>() % 50 + 10 ));
        let path = PathBuf::from("./assets/290.png");

        // let img = image::DynamicImage::new_rgb8(100, 100);
        // // 随机大小的正方形，边长在10到59之间
        // let square_size = rand::random::<u32>() % 50 + 10; 
        // let x = (100 - square_size) / 2;
        // let y = (100 - square_size) / 2;
        // let mut img: ImageBuffer<image::Rgba<u8>, Vec<u8>> = img.into_rgba8();
        // // 绘制白色正方形
        // img.put_pixel(x, y, image::Rgba([255, 255, 255, 255])); 
        // img.save(&path).unwrap();

        // 从文件中读取图像并提取ORB关键点描述符
        let n_keypoints = 5;
        let descs = load_img_get_kps(&path, n_keypoints).unwrap();
        assert_eq!(descs.len(),n_keypoints);
        // 创建一个测试词汇
        let vocab = Vocabulary::create(&descs, 3, 3);
        println!("vocab:{:?}", vocab);

        // 保存为 DBoW3 格式的 YAML 文件
        let file_path_yml = format!("./result/vocab_dbow3_{}.yml",rand::random::<u32>() % 50 + 10);
        assert!(vocab.save_dbow3(&file_path_yml).is_ok());

        // 从 DBoW3 格式的 YAML 文件加载词典
        let loaded_vocab_yml = Vocabulary::load_dbow3(file_path_yml);
        assert!(loaded_vocab_yml.is_ok());

        // 保存为 DBoW3 格式的 YAML 文件并压缩为 .gz 文件
        let file_path_gz = format!("./result/vocab_dbow3_{}.yml.gz",rand::random::<u32>() % 50 + 10);
        assert!(vocab.save_dbow3(&file_path_gz).is_ok());

        // 从 DBoW3 格式的 YAML 文件加载词典
        let loaded_vocab_gz = Vocabulary::load_dbow3(file_path_gz);
        assert!(loaded_vocab_gz.is_ok());

        // 清理测试字典文件
        // std::fs::remove_file(file_path_gz).unwrap();
        // std::fs::remove_file(file_path_yml).unwrap();

    }

}