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
#![allow(unused_doc_comments)]

//! 从点云数据中重建三维表面并生成网格:
//! 1. **加载点云数据**：程序首先从PCD文件中加载点云数据，这些数据包含了三维空间中的点及其颜色信息。
//! 2. **表面重建**：使用移动最小二乘法（MLS）对点云进行平滑处理，并计算每个点的法向量。这一步生成了带有法向量的点云数据，称为“表面元素”（surfels）。
//! 3. **网格生成**：通过贪婪投影三角化算法（Greedy Projection Triangulation）将表面元素转换为三角网格。这个算法会根据点的位置和法向量生成三角形，从而构建出三维表面。
//! 4. **可视化**：最后，程序使用PCL的可视化工具将生成的网格显示出来，用户可以通过交互方式查看三维模型。

use std::fs::create_dir_all;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Cursor, Write};
use std::path::Path;

use std::cell::{Ref, RefCell, RefMut};

// 线性代数
use nalgebra::{
    Const, DMatrix, DVector, Isometry3, Matrix, Matrix2, Matrix3, Matrix3x1, Matrix6, Point3,
    Quaternion, Rotation3, SVector, Translation3, UnitQuaternion, Vector2, Vector3, Vector6,
    VectorView3, ViewStorage,
};

// 李代数
use factrs::{
    assign_symbols,
    containers::Key,
    core::{BetweenResidual, GaussNewton, Graph, Values},
    dtype, fac,
    linalg::{DiffResult, ForwardProp, MatrixX, Numeric, NumericalDiff, VectorX},
    noise::GaussianNoise,
    optimizers::LevenMarquardt,
    residuals::{Residual1, Residual2},
    traits::*,
    variables::{MatrixLieGroup, VectorVar2, VectorVar3, SE2, SE3, SO2, SO3},
};

// 图像处理
use image::{
    buffer::ConvertBuffer, DynamicImage, GenericImageView, GrayImage, ImageBuffer, Luma, Rgb,
    RgbImage,
};
use imageproc::drawing::{draw_cross_mut, draw_filled_circle_mut, draw_line_segment_mut};

// 随机数
use rand::Rng;
use rand_distr::{Distribution, Normal};

// 点云处理
use bye_pcl_rs::{
    common::{PointXYZRGB, PointCloud, PointXYZRGBNormal},
    io::pcd_io,
};

fn main() {
    // 1. 加载ply点云文件(pcd文件解析有问题)到点云结构体
    let path = "./assets/ch12-office-xyzrgb/office1.ply";
    let result = PointCloud::<PointXYZRGB>::load_from_ply(path);
    // 断言加载成功
    assert!(result.is_ok(), "加载PLY文件失败: {:?}", result.err());
    let point_cloud = result.unwrap();
    // 验证点云数据不为空
    assert!(!point_cloud.points.is_empty(), "点云数据为空");
    
    // 2. 获取点云数量
    println!("成功加载 {} 个点", point_cloud.points.len());

    // 3. 计算点云面元素
    //
    //
    // 4. 贪婪三角化
    //

    // 5. 显示点云网格
    //
    //
}
