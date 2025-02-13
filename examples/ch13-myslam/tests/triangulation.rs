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

//! 三角测量测试

// 内部库
use myslam::preclude::*;
use myslam::algorithm::{
    triangulation, to_vec2,
};

use nalgebra::{Point2, Vector3};

/// 测试 triangulation 函数
#[test]
fn test_triangulation() {
    // 定义真实的世界坐标点
    let pt_world = Vector3::new(30.0, 20.0, 10.0);
    let mut pt_world_estimated = Vector3::zeros();

    // 定义一组位姿
    let poses = vec![
        SE3::from_rot_trans(SO3::from_xyzw(1.0, 0.0, 0.0, 0.0), Vector3::zeros()),
        SE3::from_rot_trans(SO3::from_xyzw(1.0, 0.0, 0.0, 0.0), Vector3::new(0.0, -10.0, 0.0)),
        SE3::from_rot_trans(SO3::from_xyzw(1.0, 0.0, 0.0, 0.0), Vector3::new(0.0, 10.0, 0.0)),
    ];

    // 计算归一化平面上的点
    let mut points = Vec::new();
    for pose in &poses {
        let pc = pose.apply(pt_world.fixed_view(0, 0));
        let pc_normalized = pc / pc.z;
        points.push(pc_normalized);
    }

    // 调用 triangulation 函数进行三角测量
    let result = triangulation(&poses, &points, &mut pt_world_estimated);

    // 断言三角测量成功
    assert!(result);

    // 断言估计的世界坐标点与真实值接近
    assert!((pt_world[0] - pt_world_estimated[0]).abs() < 0.01);
    assert!((pt_world[1] - pt_world_estimated[1]).abs() < 0.01);
    assert!((pt_world[2] - pt_world_estimated[2]).abs() < 0.01);
}
