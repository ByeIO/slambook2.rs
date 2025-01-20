#![allow(unused_imports)]
#![allow(deprecated)]
#![allow(unused_variables)]

// 引入 Rust 的标准库
use std::time::{SystemTime, UNIX_EPOCH};
use rand::distributions::Distribution;
// 引入 nalgebra 库的相关组件
use nalgebra::{Matrix3, Matrix, U3, DMatrix, Const};

fn main() {
    // 获取一个高质量的随机种子
    let rd = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
    let mut rng = rand::thread_rng(); // 初始化随机数生成器
    let distribution = rand::distributions::Uniform::new(-1.0, 1.0); // 均匀分布 [-1, 1]

    // 设置矩阵的大小
    let matrix_size = 5;

    // 初始化 nalgebra 矩阵
    let big_matrix = DMatrix::from_fn(matrix_size, matrix_size, |i, j| {
        distribution.sample(&mut rng) // 使用闭包填充每个元素
    });
    println!("The big matrix:\n{:?}", big_matrix);

    // 从(0,0)开始提取 3x3 的矩阵块
    let extracted_block = big_matrix.fixed_view::<3,3>(0, 0);
    println!("The extracted matrix block:\n{:?}", extracted_block);

    // 将提取的矩阵块设置为单位矩阵
    let mut assigned_block = Matrix3::identity();
    for i in 0..3 {
        for j in 0..3 {
            assigned_block[(i, j)] = extracted_block[(i, j)];
        }
    }
    println!("The assigned matrix block:\n{:?}", assigned_block);
}

