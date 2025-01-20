#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(unused_assignments)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] 
#![allow(rustdoc::missing_crate_level_docs)]
#![allow(unsafe_code)]
#![allow(clippy::undocumented_unsafe_blocks)]

use nalgebra::{
    Matrix3, Vector3, UnitQuaternion, 
    Quaternion, Isometry3, Rotation3, UnitComplex, Rotation2, Unit,
    Translation3, Perspective3, Orthographic3, Vector4, Point3, Const,
    ArrayStorage, Matrix4, ViewStorage
};

fn main() {
    // 定义两个四元数
    let q1 = Quaternion::new(0.1, 0.35, 0.2, 0.3);
    let q2 = Quaternion::new(0.2, -0.5, 0.4, -0.1);

    // 标准化四元数并转换为单位四元数
    let q1_normalized = UnitQuaternion::from_quaternion(q1.normalize());
    let q2_normalized = UnitQuaternion::from_quaternion(q2.normalize());

    // 定义两个平移向量
    let t1 = Translation3::new(0.3, 0.1, 0.1);
    let t2 = Translation3::new(-0.1, 0.5, 0.3);

    // 定义一个点
    let p1 = Vector3::new(0.5, 0.0, 0.2);

    // 创建两个等距变换
    let mut T1w = Isometry3::from_parts(t1, q1_normalized);
    let mut T2w = Isometry3::from_parts(t2, q2_normalized);

    // 计算点p1在变换T1w和T2w下的新位置
    let p2 = T2w * T1w.inverse() * p1;

    // 输出结果
    println!("{}", p2.transpose());
}
