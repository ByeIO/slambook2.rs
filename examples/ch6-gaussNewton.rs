#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(unused_assignments)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] 
#![allow(rustdoc::missing_crate_level_docs)]
#![allow(unsafe_code)]
#![allow(clippy::undocumented_unsafe_blocks)]
#![allow(unused_must_use)]
#![allow(non_snake_case)]

extern crate nalgebra as na;
extern crate rand;

use na::{
    DMatrix, DVector, Matrix3, Vector3
};

use rand::Rng;
use rand_distr::{
    Distribution, Normal
};

use std::time::{
    Instant, Duration
};

fn main() {
    // 真实参数值
    let ar = 1.0;
    let br = 2.0;
    let cr = 1.0;

    // 估计参数值
    let mut ae = 2.0;
    let mut be = -1.0;
    let mut ce = 5.0;

    // 数据点数量
    let n = 100;

    // 噪声Sigma值
    let w_sigma = 1.0;
    let inv_sigma = 1.0 / w_sigma;

    // 产生随机数
    let mut rng = rand::thread_rng();
    let normal = Normal::new(0.0, w_sigma * w_sigma);

    // 数据
    let mut x_data = Vec::new();
    let mut y_data = Vec::new();

    for i in 0..n {
        let x = i as f64 / 100.0;
        x_data.push(x);
        y_data.push((ar * x * x + br * x + cr).exp() + normal.expect("REASON").sample(&mut rng));
    }

    // 开始Gauss-Newton迭代
    // 迭代次数
    let iterations = 100; 
    let mut cost = 0.0;
    let mut last_cost = 0.0;

    let t1 = Instant::now();

    for iter in 0..iterations {
        // Hessian = J^T W^{-1} J in Gauss-Newton
        let mut h = Matrix3::zeros();
        // bias
        let mut b = Vector3::zeros(); 
        cost = 0.0;

        for i in 0..n {
            let xi = x_data[i];
            let yi = y_data[i];
            let error = yi - (ae * xi * xi + be * xi + ce).exp();
            // 雅可比矩阵
            let mut j = Vector3::zeros(); 
            // de/da
            j[0] = -xi * xi * (ae * xi * xi + be * xi + ce).exp(); 
            // de/db
            j[1] = -xi * (ae * xi * xi + be * xi + ce).exp(); 
            // de/dc
            j[2] = -(ae * xi * xi + be * xi + ce).exp(); 

            h += inv_sigma * inv_sigma * j * j.transpose();
            b += -inv_sigma * inv_sigma * error * j;

            cost += error * error;
        }

        // 求解线性方程 Hx=b
        let dx = h.cholesky().unwrap().solve(&b);
        if dx[0].is_nan() {
            println!("result is nan!");
            break;
        }

        if iter > 0 && cost >= last_cost {
            println!("cost: {} >= last cost: {}, break.", cost, last_cost);
            break;
        }

        ae += dx[0];
        be += dx[1];
        ce += dx[2];

        last_cost = cost;

        println!(
            "total cost: {}, \t\tupdate: {:?}\t\testimated params: {}, {}, {}",
            cost, dx, ae, be, ce
        );
    }

    let t2 = Instant::now();
    let time_used = t2.duration_since(t1);

    println!("solve time cost = {:?} seconds.", time_used);
    println!("estimated abc = {}, {}, {}", ae, be, ce);
}