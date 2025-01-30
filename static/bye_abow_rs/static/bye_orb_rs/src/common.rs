#![allow(dead_code)] // 允许未使用的代码
#![allow(unused_variables)] // 允许未使用的变量
#![allow(unused_imports)] // 允许未使用的导入
#![allow(unused_mut)] // 允许未使用的可变变量
#![allow(unused_assignments)] // 允许未使用的赋值
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // 在非调试模式下设置Windows子系统
#![allow(rustdoc::missing_crate_level_docs)] // 允许缺少crate级别的文档
#![allow(unsafe_code)] // 允许使用unsafe代码
#![allow(clippy::undocumented_unsafe_blocks)] // 允许未文档化的unsafe块
#![allow(unused_must_use)] // 允许未使用的must_use结果
#![allow(non_snake_case)] // 允许非蛇形命名

use bitvector::BitVector;

pub type Point = (i32, i32);
pub type IndexMatch = (usize, usize);

/// 定义一个可匹配的特征，要求实现计算距离的方法
pub trait Matchable {
    /// 计算两个特征之间的距离
    ///
    /// # 参数
    ///
    /// * `other` - 另一个特征
    ///
    /// # 返回值
    ///
    /// 返回两个特征之间的距离
    fn distance(&self, other: &Self) -> usize;
}

/// 匹配两个向量中的索引
///
/// # 参数
///
/// * `vec1` - 第一个向量
/// * `vec2` - 第二个向量
///
/// # 返回值
///
/// 返回匹配的索引对的向量
pub fn match_indices<T>(vec1: &Vec<T>, vec2: &Vec<T>) -> Vec<IndexMatch>
where
    T: Matchable
{
    // assert_eq!(vec1.len(), vec2.len());
    if vec1.len() != vec2.len(){
        let nil : Vec<IndexMatch> = vec![];
        return nil;
    }

    let mut index_vec = vec![];
    let len = vec1.len();
    let mut matched_indices = BitVector::new(len);

    for i in 0..len {
        let mut min_dist:usize = usize::MAX;
        let mut matched_index:usize = 0;
        for j in 0..len {
            if matched_indices.contains(j) { 
                continue
            }

            let dist = vec1[i].distance(&vec2[j]);
            if dist < min_dist {
                min_dist = dist;
                matched_index = j;
            }
        }

        index_vec.push((i as usize, matched_index as usize));
        matched_indices.insert(matched_index);
    }

    index_vec
}

/// 自适应非最大抑制
///
/// # 参数
///
/// * `vec` - 输入的特征向量
/// * `n` - 需要保留的最大特征数
///
/// # 返回值
///
/// 返回抑制后的特征向量
pub fn adaptive_nonmax_suppression<T>(vec: &mut Vec<T>, n: usize) -> Vec<T> 
where
    T: Matchable,
    T: Copy
{
    if n>vec.len(){
        let nil : Vec<T> = vec![];
        return nil;
    }
    // assert!(n <= vec.len());

    let mut maximal_keypoints:Vec<T> = vec![];
    for i in 1..vec.len() - 1 {
        let d1 = &vec[i];
        let mut min_dist:usize = usize::MAX;
        let mut min_idx:usize = 0;

        for j in 0..i {
            let d0 = &vec[j];
            let dist = d0.distance(&d1);
            if dist < min_dist {
                min_dist = dist;
                min_idx = j;
            }
        }

        vec.swap(i, min_idx);
    }

    for k in 0..n {
        maximal_keypoints.push(vec[k]);
    }

    maximal_keypoints
}