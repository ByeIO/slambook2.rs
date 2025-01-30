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
#![allow(unreachable_patterns)]

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

// Cluster元组
pub enum ClusterInitMethod {
    Random,
    KMeansPP,
}

/// 由图像特征集合构建的视觉词汇表。
#[derive(Serialize, Deserialize, PartialEq, Clone, Default)]
pub struct Vocabulary {
    pub blocks: Vec<Block>,
    pub k: usize,
    pub levels: usize,
    pub num_blocks: usize,
    pub num_leaves: usize,
}

/// Vocabulary的实现第一部分(公开API)
impl Vocabulary {
    /// 将二进制描述符向量转换为相对于词汇表的词袋表示。
    /// 描述符进行 L1 归一化。
    /// 如果特征为空，则返回 Err。
    pub fn transform(&self, features: &[Desc]) -> BowResult<BoW> {
        self.transform_inner(features, false).map(|res| res.0)
    }

    /// 将二进制描述符向量转换为相对于词汇表的词袋表示。
    /// 描述符进行 L1 归一化。
    /// 如果特征为空，则返回 Err。
    ///
    /// 同时提供从特征到词汇表树中对应节点的“直接索引”。
    ///
    /// 特征 `feature[i]` 的直接索引为 `di = DirectIdx[i]`，其中
    /// `di.len() <= l`（层数），`di[j]` 是匹配 `feature[i]` 的节点
    /// 在词汇表树中第 `j` 层的 id。
    pub fn transform_with_direct_idx(&self, features: &[Desc]) -> BowResult<(BoW, DirectIdx)> {
        self.transform_inner(features, true)
    }

    /// 从描述符集合构建词汇表。
    ///
    /// 参数：
    /// - k：分支因子
    /// - l：最大层数（应 <= 5）
    pub fn create(features: &[Desc], k: usize, l: usize) -> Self {
        // 从树的根节点开始
        let mut v = Self::empty(k, l);

        // 使用递归 k 均值聚类构建特征
        v.cluster(features, vec![0], 1);

        // 按块 id 排序
        v.blocks.sort_by(|a, b| a.id.get_bid().cmp(&b.id.get_bid()));

        v
    }

    /// 从文件加载 ABoW 词汇表
    pub fn load<P: AsRef<std::path::Path>>(file: P) -> BowResult<Self> {
        let mut file = std::fs::File::open(file)?;
        let mut buffer: Vec<u8> = Vec::new();
        std::io::Read::read_to_end(&mut file, &mut buffer)?;
        Ok(bincode::deserialize(&buffer)?)
    }

    /// 将词汇表保存到文件
    pub fn save<P: AsRef<std::path::Path>>(&self, file: P) -> BowResult<()> {
        let serialized = bincode::serialize(&self)?;
        let mut file = std::fs::File::create(file)?;
        std::io::Write::write_all(&mut file, &serialized)?;
        Ok(())
    }
}

/* start 数据结构 */

/// 表示词汇表中非叶子节点的单元
#[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
pub struct Block {
    pub id: NodeId,
    pub children: Children,
}

/// 表示块的子节点的数据结构，可能是叶子节点，也可能不是
#[derive(Serialize, Deserialize, PartialEq, Clone)]
pub struct Children {
    pub features: Vec<Desc>,
    pub weights: Vec<f32>,
    pub cluster_size: Vec<usize>,
    pub ids: Vec<NodeId>,
}

/// 节点的唯一标识符。Leaf 变体存储其所有父节点的 id，
/// 这相当于匹配该叶子的任何特征的 DirectIndex。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum NodeId {
    Block(usize),
    Leaf(IdPath),
}

/* end 数据结构 */

/// Vocabulary的实现第二部分(私有)
impl Vocabulary {
    pub fn transform_inner(&self, features: &[Desc], di: bool) -> BowResult<(BoW, DirectIdx)> {
        if features.is_empty() {
            return Err(BowErr::NoFeatures);
        }

        let mut bow = BoW(vec![0.; self.num_leaves]);
        let mut direct_idx: DirectIdx = Vec::with_capacity(features.len());
        for feature in features {
            // 从根块开始
            let mut block = &self.blocks[0];

            // 遍历树
            loop {
                let mut best_child: (u8, usize) = (u8::MAX, 0);
                for (child, child_feat) in block.children.features.iter().enumerate() {
                    let d = hamming(feature, child_feat);
                    if d < best_child.0 {
                        best_child = (d, child)
                    }
                }
                match &block.children.ids[best_child.1] {
                    NodeId::Block(id) => {
                        println!("当前 block id: {}, 最大 block 数: {}", *id, self.blocks.len());
                        // FIXME: 使用 min 限制索引范围，防止越界
                        let safe_id = (*id).min(self.blocks.len() - 1);
                        block = &self.blocks[safe_id];
                    }
                    NodeId::Leaf(ids) => {
                        if di {
                            // 将词父节点 id 添加到直接索引中
                            direct_idx.push(ids.clone());
                        }
                        // 将词/叶子 id 和权重添加到结果中
                        let word_id = *ids.last().unwrap();
                        let weight = block.children.weights[best_child.1];
                        match bow.0.get_mut(word_id) {
                            Some(w) => *w += weight,
                            None => bow.0[word_id] = weight,
                        }
                        break;
                    }
                }
            }
        }
        // 归一化 BoW 向量
        let sum: f32 = bow.0.iter().sum();
        if sum > 0. {
            let inv_sum = 1. / sum;
            for w in bow.0.iter_mut() {
                *w *= inv_sum;
            }
        }

        Ok((bow, direct_idx))
    }

    pub fn cluster(&mut self, features: &[Desc], parent_ids: Vec<usize>, curr_level: usize) {
        // println!(
        //     "KMeans step with {} features. parents: {:?}, level {}",
        //     features.len(),
        //     parent_ids,
        //     curr_level
        // );

        let mut clusters = self.initialize_clusters(features, ClusterInitMethod::KMeansPP);
        let mut groups = vec![Vec::new(); clusters.len()];

        loop {
            let mut new_groups: Vec<Vec<usize>> = vec![Vec::new(); groups.len()];
            for (i, f) in features.iter().enumerate() {
                let mut best: (usize, u8) = (0, u8::MAX);
                for (j, c) in clusters.iter().enumerate() {
                    let d = hamming(c, f);
                    if d < best.1 {
                        best = (j, d);
                    }
                }
                new_groups[best.0].push(i);
            }

            if groups == new_groups {
                break; // 收敛
            }

            // 更新聚类中心
            clusters = new_groups
                .iter()
                .map(|group| {
                    let desc = group.iter().map(|&i| &features[i]).collect();
                    Self::desc_mean(desc)
                })
                .collect();
            groups = new_groups;
        }

        // 移除罕见的空组
        groups.retain(|g| !g.is_empty());
        clusters.retain(|c| c != &[0_u8; std::mem::size_of::<Desc>()]);
        assert_eq!(groups.len(), clusters.len());

        // 创建块
        let ids: Vec<_> = groups
            .iter()
            .map(|g| self.next_node_id(curr_level == self.levels || g.len() == 1, &parent_ids))
            .collect();
        let children = Children {
            weights: vec![1.; groups.len()],
            ids: ids.clone(),
            cluster_size: groups.iter().map(|g| g.len()).collect(),
            features: clusters,
        };
        let block = Block {
            id: NodeId::Block(*parent_ids.last().unwrap()),
            children,
        };
        self.blocks.push(block);

        // 递归
        if curr_level < self.levels {
            for (i, id) in ids
                .iter()
                .enumerate()
                .filter(|&(_, n)| matches!(n, NodeId::Block(_)))
            {
                // 获取子聚类的特征
                let features: Vec<Desc> = groups[i].iter().map(|&j| features[j]).collect();

                // 更新父节点 id
                let mut ids = parent_ids.clone();
                ids.push(id.get_bid());

                // 对子特征进行聚类
                self.cluster(&features, ids, curr_level + 1);
            }
        }
    }

    /// 为 k 均值聚类初始化聚类中心
    pub fn initialize_clusters(&self, features: &[Desc], method: ClusterInitMethod) -> Vec<Desc> {
        // 如果特征数量少于 k，则直接返回这些特征
        if features.len() <= self.k {
            return features.to_vec();
        }

        let mut deduped = features.to_vec();
        deduped.sort_unstable();
        deduped.dedup();

        if deduped.len() <= self.k {
            return deduped;
        }

        match method {
            ClusterInitMethod::Random => self.init_random(features),
            ClusterInitMethod::KMeansPP => self.init_kmeanspp(features),
        }
    }

    pub fn init_random(&self, features: &[Desc]) -> Vec<Desc> {
        let mut rng = rng();
        features
            .choose_multiple(&mut rng, self.k)
            .cloned()
            .collect()
    }

    pub fn init_kmeanspp(&self, features: &[Desc]) -> Vec<Desc> {
        let mut rng = rng();
        let mut features = features.to_owned();
        let mut centroids = Vec::with_capacity(self.k);
        // 1. 随机选择第一个聚类中心。
        let random_idx = rng.random_range(0..features.len());
        centroids.push(features.remove(random_idx));

        while centroids.len() < self.k {
            // 2. 对于每个数据点，计算其与最近的、先前选择的聚类中心的距离。
            let mut dists: Vec<f32> = vec![std::u8::MAX as f32; features.len()];
            for (i, f) in features.iter().enumerate() {
                for c in centroids.iter() {
                    dists[i] = f32::min(hamming(f, c) as f32, dists[i]);
                }
            }
            // 3. 选择下一个聚类中心，选择点作为聚类中心的概率与其距离最近的、先前选择的聚类中心的距离成正比。
            let centroid_weights = WeightedIndex::new(dists).expect("weighted index err");
            let weighted_random_idx = centroid_weights.sample(&mut rng);
            centroids.push(features.remove(weighted_random_idx));
        }

        centroids
    }

    #[inline]
    /// 计算二进制数组（描述符）集合的均值。
    pub fn desc_mean(descriptors: Vec<&Desc>) -> Desc {
        let n2 = descriptors.len() / 2;
        let mut counts = vec![0; std::mem::size_of::<Desc>() * 8];
        let mut result: Desc = [0; std::mem::size_of::<Desc>()];
        let result_bits = result.view_bits_mut::<Msb0>();
        for d in descriptors {
            for (i, b) in d.view_bits::<Msb0>().iter().enumerate() {
                if *b {
                    counts[i] += 1;
                }
            }
        }
        for (i, &c) in counts.iter().enumerate() {
            if c > n2 {
                result_bits.set(i, true);
            }
        }
        result
    }

    /// 提供下一个 NodeId，可能是叶子/词，也可能是块。
    pub fn next_node_id(&mut self, leaf: bool, parent_ids: &[usize]) -> NodeId {
        if leaf {
            // 叶子节点将存储其父节点的块 id，以便于后续获取直接索引
            let mut new_parent_ids = parent_ids[1..].to_smallvec(); // 去掉第一个父节点，它总是 0
            new_parent_ids.push(self.num_leaves); // 添加叶子 id
            self.num_leaves += 1;
            NodeId::Leaf(new_parent_ids)
        } else {
            self.num_blocks += 1;
            NodeId::Block(self.num_blocks)
        }
    }
    pub fn empty(k: usize, l: usize) -> Self {
        Self {
            blocks: Vec::new(),
            k,
            num_blocks: 0,
            num_leaves: 0,
            levels: l,
        }
    }
}

/* start 辅助函数 */
/// 计算两个二进制数组（描述符）之间的汉明距离。
#[inline]
pub fn hamming(x: &[u8], y: &[u8]) -> u8 {
    x.iter()
        .zip(y)
        .fold(0, |a, (b, c)| a + (*b ^ *c).count_ones() as u8)
}

impl NodeId {
    pub fn get_bid(&self) -> usize {
        match self {
            NodeId::Block(i) => *i,
            NodeId::Leaf(ids) => *ids.last().unwrap(), // 返回叶子节点的最后一个 ID
            NodeId::Leaf(_) => unreachable!(),
        }
    }
}

impl fmt::Debug for Children {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Children")
            .field("ids", &self.ids)
            .field("weights", &self.weights)
            .field("cluster size", &self.cluster_size)
            .finish()
    }
}

impl fmt::Debug for Vocabulary {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut clust_sizes: Vec<usize> = Vec::new();
        for b in self.blocks.iter() {
            for (i, &c) in b.children.cluster_size.iter().enumerate() {
                if matches!(b.children.ids[i], NodeId::Leaf(_)) {
                    clust_sizes.push(c);
                }
            }
        }
        let sum = clust_sizes.iter().sum::<usize>();
        clust_sizes.sort_unstable();
        f.debug_struct("Vocabulary")
            .field("Word/Leaf Nodes", &self.num_leaves)
            .field("Other Nodes", &self.num_blocks)
            .field("Levels", &self.levels)
            .field("Branching Factor", &self.k)
            .field("Total Training Features", &sum)
            .field(
                "Min Word Cluster Size",
                &(clust_sizes.iter().min().unwrap()),
            )
            .field(
                "Max Word Cluster Size",
                &(clust_sizes.iter().max().unwrap()),
            )
            .field("Mean Word Cluster Size", &(sum / clust_sizes.len()))
            .field(
                "Median Word Cluster Size",
                &clust_sizes[clust_sizes.len() / 2],
            )
            .finish()
    }
}
/* end 辅助函数 */

#[cfg(test)]
mod tests1 {
    use super::*;
    #[test]
    fn test_transform_empty_features() {
        let vocab = Vocabulary::empty(3, 3);
        let features: Vec<Desc> = vec![];
        assert!(vocab.transform(&features).is_err());
    }

    #[test]
    fn test_transform_non_empty_features() {
        use crate::keypoint::load_img_get_kps;
        use image::ImageBuffer;
        use std::path::PathBuf;
        // 创建一个测试图像, 仅支持png格式
        let path = PathBuf::from(format!("./test_img_{}.png", rand::random::<u32>() % 50 + 10 ));
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
        assert_eq!(descs.len(),n_keypoints);
        // 创建一个测试词汇
        let vocab = Vocabulary::create(&descs, 3, 3);
        // 示例特征
        let features: Vec<Desc> = vec![[1u8; 32]; 10]; 
        let bow = vocab.transform(&features);
        // 判断转换成功与否
        assert!(bow.is_ok());
        let bow = bow.unwrap();
        assert_eq!(bow.0.len(), vocab.num_leaves);
        // 删除测试图像文件
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn test_create_vocabulary() {
        // 示例特征
        let features: Vec<Desc> = vec![[1u8; 32]; 100]; 
        let vocab = Vocabulary::create(&features, 3, 3);
        assert!(!vocab.blocks.is_empty());
        assert_eq!(vocab.k, 3);
        assert_eq!(vocab.levels, 3);
    }

    #[test]
    fn test_save_and_load_vocabulary() {
        // 示例特征
        let features: Vec<Desc> = vec![[1u8; 32]; 100];
        let vocab = Vocabulary::create(&features, 3, 3);
        let file_path = "test_vocab.bin";
        assert!(vocab.save(file_path).is_ok());
        let loaded_vocab = Vocabulary::load(file_path);
        assert!(loaded_vocab.is_ok());
        assert_eq!(vocab, loaded_vocab.unwrap());
        // 清理测试文件
        std::fs::remove_file(file_path).unwrap(); 
    }

}