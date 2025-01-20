// ch3-useNalgebra
// 稠密矩阵的代数运算(逆,特征值等)
// 测试Matrix,Vector3,Vector2,OVector基本类型的使用

extern crate nalgebra as na;

extern crate rand; // 确保在Cargo.toml中添加了rand依赖
use rand::Rng; // 引入Rng trait以使用随机数生成功能

use na::{Matrix3, Matrix, Vector3, OVector, Dyn, SymmetricEigen, SMatrix, Const};
use std::time::Instant;

// 矩阵大小
const MATRIX_SIZE: usize = 50;

// examples单文件都要有main函数
fn main() {
    // 声明一个 2x3 的 f32 矩阵
    let matrix_23 = na::Matrix2x3::<f32>::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    println!("matrix 2x3 from 1 to 6:\n{}", matrix_23);

    // 用索引访问矩阵中的元素
    println!("print matrix 2x3:");
    for i in 0..2 {
        for j in 0..3 {
            print!("{}\t", matrix_23[(i, j)]);
        }
        println!();
    }

    // 矩阵和向量相乘
    let v_3d = Vector3::new(3.0, 2.0, 1.0);
    let vd_3d = Vector3::new(4.0, 5.0, 6.0);

    // 显式转换
    let result: na::Vector2<f64> = matrix_23.map(|x| x as f64) * v_3d;
    println!("[1,2,3;4,5,6]*[3,2,1]={}", result.transpose());

    let result2 = matrix_23 * vd_3d;
    println!("[1,2,3;4,5,6]*[4,5,6]: {}", result2.transpose());

    // 生成3阶整数随机方阵
    // pub fn new_random(nrows: usize, ncols: usize) -> Self
    // Matrix3为3阶方阵
    // let matrix_33 = Matrix3::<f64>::new_random(); // 生成的全是小数
    let matrix_33 = Matrix3::from_fn(|_, _| rand::random::<i32>() as f64);
    println!("random matrix:\n{}", matrix_33);

    // 矩阵转置
    println!("transpose:\n{}", matrix_33.transpose());
    // 矩阵各元素之和
    println!("sum: {}", matrix_33.sum());
    // 矩阵的迹
    println!("trace: {}", matrix_33.trace());
    // 矩阵倍乘
    println!("times 10:\n{}", 10.0 * matrix_33);
    // 矩阵求逆
    println!("inverse:\n{}", matrix_33.try_inverse().unwrap());
    // 矩阵行列式
    println!("det: {}", matrix_33.determinant());

    // 特征值
    // 实对称矩阵可以保证对角化成功
    let eigen_solver = matrix_33.clone().transpose() * matrix_33.clone();
    // 计算对称矩阵的特征值和特征向量
    let eigen = eigen_solver.symmetric_eigen();

    // 获取特征值和特征向量
    let eigen_values = eigen.eigenvalues;
    let eigen_vectors = eigen.eigenvectors;

    println!("Eigen values = \n{:?}", eigen_values);
    println!("Eigen vectors = \n{}", eigen_vectors);

    // 解方程
    // 求解matrix_NN * x = v_Nd方程
    let mut matrix_NN = SMatrix::<f64, MATRIX_SIZE, MATRIX_SIZE>::from_fn(|_, _| rand::random::<i32>() as f64);
    matrix_NN = matrix_NN * matrix_NN.transpose(); // 保证半正定
    let v_Nd = OVector::<f64, Const<MATRIX_SIZE> >::new_random();

    // 记录开始时间
    let start_time = Instant::now();

    // 直接求逆方式解方程
    let x = matrix_NN.try_inverse().unwrap() * v_Nd;
    println!("time of normal inverse is {}ms", start_time.elapsed().as_millis());
    println!("x = {}", x.transpose());

    // QR 分解方式解方程
    let start_time = Instant::now();
    let x = matrix_NN.qr().solve(&v_Nd).unwrap();
    println!("time of QR decomposition is {}ms", start_time.elapsed().as_millis());
    println!("x = {}", x.transpose());

    // Cholesky 分解方式解方程
    let start_time = Instant::now();
    let x = matrix_NN.cholesky().unwrap().solve(&v_Nd);
    println!("time of Cholesky decomposition is {}ms", start_time.elapsed().as_millis());
    println!("x = {}", x.transpose());
}

