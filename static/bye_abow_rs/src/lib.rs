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

use smallvec::SmallVec;
use thiserror::Error;
use serde::{
    Deserialize, Serialize
};
use serde_yml::Error;
use std::num::ParseIntError;

/// vocabulary模块
pub mod vocabulary;

/// adaptor模块
pub mod adaptor;

/// keypoint模块
pub mod keypoint;

/// database模块
pub mod database;

/// 支持的描述符类型是256位数组
pub type Desc = [u8; 32];

/// 图像或描述符集的词袋表示。
/// 索引: 词汇表中的单词/叶子ID。
/// 值: 在提供的特征中该单词的总权重。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoW(pub Vec<f32>);

/// 从特征到词汇树中对应节点的映射。
/// 每个特征映射到多个节点，每个树层最多一个。
/// `feature[i]`的直接索引是`di = DirectIdx[i]`，其中
/// `di.len() <= l`（层数）。
/// `di[j]`是与`feature[i]`匹配的节点在词汇树中
/// 第`j`层的ID。
pub type DirectIdx = Vec<IdPath>;

/// 给定特征从根到叶子的路径。
/// 只有5个条目是堆栈分配的，因此层数>5时性能较差。
pub type IdPath = SmallVec<[usize; 5]>;

/// BoW的实现
impl BoW {
    /// 计算两个BoW之间的L1范数。（用于Galvez（方程2））。
    pub fn l1(&self, other: &Self) -> f32 {
        let values = self.0.iter().zip(&other.0);
        1. - 0.5 * (values.fold(0., |a, (b, c)| a + (b - c).abs()))
    }
}

/// 定义报错类型
type BowResult<T> = std::result::Result<T, BowErr>;
#[derive(Error, Debug)]
pub enum BowErr {
    // 未提供任何特征
    #[error("No Features Provided")]
    NoFeatures,

    // IO错误
    #[error("Io Error")]
    Io(#[from] std::io::Error),

    // 序列化错误
    #[error("Vocabulary Serialization Error")]
    Bincode(#[from] bincode::Error),

    // yaml错误
    #[error("YAML Serialization Error")]
    Yaml(#[from] serde_yml::Error),

    // 解析错误
    #[error("Parse Error: {0}")]
    ParseError(String),

    // 整数解析错误
    #[error("Parse Int Error: {0}")]
    ParseInt(#[from] ParseIntError),
}

// // 实现 From<ParseIntError> 转换
// impl From<ParseIntError> for BowErr {
//     fn from(err: ParseIntError) -> Self {
//         BowErr::ParseInt(err)
//     }
// }