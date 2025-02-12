#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(unused_assignments)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] 
#![allow(rustdoc::missing_crate_level_docs)]
#![allow(unsafe_code)]
#![allow(clippy::undocumented_unsafe_blocks)]

// 图优化
use factrs::{
    assign_symbols,
    core::{BetweenResidual, GaussNewton, Graph, Values},
    dtype, fac,
    linalg::{ ForwardProp, Numeric, NumericalDiff, VectorX, DiffResult, MatrixX },
    residuals::Residual1,
    traits::*,
    variables::{VectorVar2, SE2, VectorVar3, SE3, SO2, SO3, MatrixLieGroup},
    containers::Key,
    noise::{GaussianNoise},
};

use nalgebra::{
    Quaternion, Vector3, Matrix3, UnitQuaternion, 
    Isometry3, Translation3, Const,
    Matrix, Vector6, Point3, ViewStorage, Rotation3,
    Matrix3x1, VectorView3
};

use std::f64::consts::PI;
use std::f64::consts::FRAC_PI_2;

fn main() {
    // 1. 从Z轴旋转90度的旋转矩阵构造SO3
    let angle = FRAC_PI_2; // 90度
    let rotation_matrix = Matrix3::new(
        angle.cos(), -angle.sin(), 0.0,
        angle.sin(), angle.cos(), 0.0,
        0.0, 0.0, 1.0
    );
    let rotation_matrix_view = rotation_matrix.fixed_view::<3,3>(0,0);

    // 使用 from_matrix 方法构造 SO3
    let so3_matrix = SO3::from_matrix(rotation_matrix_view);
    println!("成功从旋转矩阵构造SO3: {:?}", so3_matrix);
    
    // 2. 通过四元数构造SO3
    // 使用 `UnitQuaternion` 来构造旋转
    let axis = Vector3::z_axis();
    let angle = FRAC_PI_2;
    let quaternion = UnitQuaternion::from_axis_angle(&axis, angle);

    // 提取四元数的 x, y, z, w 分量
    let x = quaternion.i;
    let y = quaternion.j;
    let z = quaternion.k;
    let w = quaternion.w;

    // 使用 from_xyzw 构造 SO3
    let so3_quaternion = SO3::from_xyzw(x, y, z, w);
    println!("成功从四元数构造SO3: {:?}", so3_quaternion);

    // 3. 使用对数映射获得SO(3)李代数向量
    let so3_log = so3_matrix.log();
    println!("so3_log = {:?}", so3_log);

    // hat为向量到反对称矩阵
    let so3_log_view = so3_log.as_view();
    let so3_hat = SO3::hat(so3_log_view);
    println!("so3 hat=\n{:?}", so3_hat);
    // 相对的,vee为反对称矩阵到向量
    let so3_vee = SO3::vee(so3_hat.fixed_view::<3,3>(0,0).into());
    println!("so3 hat vee= {:?}", so3_vee);

    // 增量扰动模型的更新,假设更新量为这么多
    let update_so3 = Vector3::new(1e-4, 0.0, 0.0);
}
