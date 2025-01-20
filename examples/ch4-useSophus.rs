#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(unused_assignments)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] 
#![allow(rustdoc::missing_crate_level_docs)]
#![allow(unsafe_code)]
#![allow(clippy::undocumented_unsafe_blocks)]

use sophus::lie::LieGroup;

use nalgebra::{
    Matrix3, Vector3, UnitQuaternion, 
    Quaternion, Isometry3, Rotation3, UnitComplex, Rotation2, Unit,
    Translation3, Perspective3, Orthographic3, Vector4, Point3, Const,
    ArrayStorage, Matrix4, ViewStorage
};

use std::f64::consts::PI;

fn main() {
    // 1. 从Z轴旋转90度的旋转矩阵构造SO(3)李代数
    // 定义旋转矩阵
    // 旋转是一种可逆的、保持原点、距离和方向的变换。在代数学家中，它通常被称为n维特殊正交群SO(n)。
    // 1.1 硬编码
    let rotation_matrix3 = nalgebra::Matrix3::new(0.0, -1.0, 0.0,
        1.0,  0.0, 0.0,
        0.0,  0.0, 1.0);
    // 1.2 现场计算
    // 定义旋转轴为Z轴
    let axis = Unit::new_normalize(Vector3::new(0.0, 0.0, 1.0));
    // 创建绕Z轴旋转的旋转矩阵
    let rotation = nalgebra::Rotation3::from_axis_angle(&axis, std::f64::consts::PI / 2.0);
    // Rotation3 转 Matrix3
    let rotation_matrix = rotation.matrix();
    // 尝试从旋转矩阵构造Rotation3 (SO3)
    let so3_r: sophus::lie::groups::rotation3::Rotation3<f64,1,0,0> = sophus::lie::groups::rotation3::Rotation3::try_from_mat(rotation_matrix).expect("无法从旋转矩阵构造Rotation3");
    println!("成功构造SO(3)的Rotation3: {:?}", so3_r);

    // 2. 从四元数构造SO(3)李代数
    let q = nalgebra::UnitQuaternion::from_rotation_matrix(&rotation);
    // 将单位四元数转换回旋转矩阵
    let rotation_matrix: nalgebra::Matrix3<f64> = q.to_rotation_matrix().into();
    let so3_q: sophus::lie::groups::rotation3::Rotation3<f64,1,0,0> = sophus::lie::groups::rotation3::Rotation3::try_from_mat(rotation_matrix).expect("无法从四元数构造Rotation3");
    println!("SO(3) from quaternion: \n{:?}", so3_q);

    // 3. 使用对数映射获得SO(3)李代数向量
    let so3 = so3_r.log();
    println!("so3 = {:?}", so3);

    // hat为向量到反对称矩阵,显式指定泛型参数和常量参数
    let so3_hat = LieGroup::<f64, 3, 4, 3, 3, 1, 0, 0, sophus_lie::groups::rotation3::Rotation3Impl<f64,1,0,0>>::hat(&so3);
    println!("so3 hat=\n{:?}", so3_hat);
    // 相对的,vee为反对称到向量
    println!("so3 hat vee= {:?}", LieGroup::<f64, 3, 4, 3, 3, 1, 0, 0, sophus_lie::groups::rotation3::Rotation3Impl<f64,1,0,0>>::vee(&so3_hat));

    // 增量扰动模型的更新,假设更新量为这么多
    let update_so3 = sophus::autodiff::nalgebra::Vector3::new(1e-4, 0.0, 0.0);
}