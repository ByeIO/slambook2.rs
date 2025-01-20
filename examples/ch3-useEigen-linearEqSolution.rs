#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
extern crate nalgebra as na;
extern crate rand;
use na::{DMatrix, DVector, Matrix3, Vector3, SVector, Matrix, VecStorage, Dyn};
use rand::{Rng, SeedableRng};
use rand::distributions::{Distribution, Uniform};
use std::process::exit;

// 自定义高斯消元法函数,任意维度
fn gaussian_elimination(a: &DMatrix<f64>, b: &DVector<f64>) -> DVector<f64> {
    let n = a.nrows(); // 矩阵的行数（即方程个数）
    // 将 A 和 b 合并为增广矩阵 [A | b]
    let mut augmented = DMatrix::from_element(n, n + 1, 0.0);
    for i in 0..n {
        for j in 0..n {
            augmented[(i, j)] = a[(i, j)];
        }
        augmented[(i, n)] = b[i];
    }
    // 消元阶段：将矩阵 A 转化为上三角矩阵
    for k in 0..n {
        // 1. 选主元：确保当前对角线元素非零
        for i in k + 1..n {
            if augmented[(k, k)] == 0.0 {
                eprintln!("Zero pivot encountered. Matrix is singular!");
                exit(1);
            }
            // 2. 消去第 k 列的第 i 行元素
            let factor = augmented[(i, k)] / augmented[(k, k)];
            for j in k..=n {
                augmented[(i, j)] -= factor * augmented[(k, j)];
            }
        }
    }
    // 回代阶段：从最后一行开始，求解未知数
    let mut x = DVector::from_element(n, 0.0);
    for i in (0..n).rev() {
        x[i] = augmented[(i, n)];
        for j in i + 1..n {
            x[i] -= augmented[(i, j)] * x[j];
        }
        x[i] /= augmented[(i, i)];
    }
    x // 返回值
}

fn main() {
    // 设置随机数种子
    let mut rng = rand::rngs::StdRng::seed_from_u64(42); 

    // 均匀分布 [-1, 1]
    let distribution = Uniform::new(-1.0, 1.0); 

    // 初始化 nalgebra 矩阵
    let b1 = Matrix3::from_fn(|_, _| distribution.sample(&mut rng));
    // a1需要为动态大小
    let a1: Matrix3<f64> = b1.transpose() * b1 + 0.1 * Matrix3::identity();
    // 使用闭包将 Matrix3 转换为 DMatrix
    let a1_dmatrix = DMatrix::from_fn(3, 3, |i, j| {
        a1[(i, j)]
    });
    println!("a1_dmatrix:{:?}", a1_dmatrix);

    // 获取特征值
    let eigenvalues = a1.eigenvalues().unwrap();
    println!("eigenValue of A =\n{:?}", eigenvalues);

    // 生成随机向量 b
    let b = DVector::from_fn(3, |_,_| distribution.sample(&mut rng));

    // 1.1 调用自定义高斯消元法（Gaussian Elimination）函数
    let gex = gaussian_elimination(&a1_dmatrix.clone().into(), &b);
    println!("Solution of Gaussian Elimination: x =\n{:?}", gex);

    // 1.2 使用直接求逆方式解Ax=b
    // let ex = a1_dmatrix.solve(&b).expect("Matrix is singular");
    let a1_inv = a1_dmatrix.try_inverse().expect("Matrix is singular");
    let ex_inv = a1_inv * b.clone();
    println!("Solution of nalgebra: x =\n{:?}", ex_inv);

    // 2. 使用LU分解法求解线性方程Ax=b
    let lux = a1.lu().solve(&b).expect("Matrix is singular");
    println!("Solution of LU decomposition: x =\n{:?}", lux);

    // 3. 使用LLT分解法(Cholesky分解)求解线性方程Ax=b，要求A是对称正定矩阵
    if let Some(lltx) = a1.cholesky() {
        println!("Solution of LLT decomposition: x =\n{:?}", lltx.solve(&b));
    } else {
        println!("Matrix A is not positive definite!");
    }

    // 4. 使用QR分解法
    let qr = a1.qr();
    let qrx = qr.solve(&b).expect("Matrix is singular");
    println!("Solution of QR decomposition: x =\n{:?}", qrx);

    // 5. 奇异值分解 (SVD) 求解
    let svd = a1.svd(true, true);
    let svdx = svd.solve(&b,1e-10).expect("Matrix is singular");
    println!("Solution of SVD: x ={:?}", svdx);
    println!("Singular values of A:\n{:?}", svd.singular_values);

    // 6. 使用特征值分解求解
    let eigen = a1.symmetric_eigen();
    let eigen_values = eigen.eigenvalues;
    let v = eigen.eigenvectors;
    let lambda = eigen.eigenvalues;

    // 计算中间变量 y = V.inverse() * b
    let mut y = v.try_inverse().expect("Matrix is not invertible") * b;

    // 通过 Lambda 求解 y，并还原 x = V * y
    let mut evdx = Vector3::zeros();
    for i in 0..3 {
        if lambda[i] != 0.0 { // 避免除以 0
            y[i] /= lambda[i];
        } else {
            eprintln!("Error: Zero eigenvalue encountered!");
            return;
        }
    }
    evdx = v * y;
    println!("Solution of Eigen Value Decomposition: x =\n{:?}", evdx);
}