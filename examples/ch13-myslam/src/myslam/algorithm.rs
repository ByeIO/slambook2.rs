#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(unused_mut)]
#![allow(unused_variables)]

//! 实用算法

// 使用类型别名
// 假设 super::preclude 存在，这里未给出具体实现，保留原代码
use super::preclude::*;

// 标准库
use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::cell::{
    RefMut, Ref, RefCell
};

use nalgebra::{Matrix3x4, SVD, Point2, Vector3, Matrix, Vector2, OMatrix};

/// 使用 SVD 进行线性三角测量
/// 
/// # 参数
/// - `poses`：位姿数组
/// - `points`：归一化平面上的点数组
/// - `pt_world`：世界坐标系中的三角测量点
/// 
/// # 返回值
/// 如果成功则返回 `true`，否则返回 `false`
pub fn triangulation(poses: &Vec<SE3>, points: &Vec<Vector3<f64>>, pt_world: &mut Vector3<f64>) -> bool {
    // 构建矩阵 A 和向量 b
    let rows = 2 * poses.len();
    let mut A = OMatrix::<f64, nalgebra::Dyn, nalgebra::U4>::zeros(rows);
    let mut b = Vector3::<f64>::zeros();

    // 填充矩阵 A
    for (i, pose) in poses.iter().enumerate() {
        let pose_matrix = pose.to_matrix();
        let m = pose_matrix.fixed_view::<3, 4>(0, 0);
        let row_0 = points[i][0] * m.row(2) - m.row(0);
        let row_1 = points[i][1] * m.row(2) - m.row(1);
        A.fixed_view_mut::<1, 4>(2 * i, 0).copy_from(&row_0.fixed_view::<1, 4>(0, 0));
        A.fixed_view_mut::<1, 4>(2 * i + 1, 0).copy_from(&row_1.fixed_view::<1, 4>(0, 0));
    }

    // 进行 SVD 分解
    let svd = A.svd(true, true);

    // 计算世界坐标系中的点
    let svd_v_t = svd.v_t.expect("REASON");
    println!("svd_v_t: {}", svd_v_t);

    // 检查矩阵维度
    if svd_v_t.nrows() < 4 {
        return false;
    }

    // 检查除数是否为零
    let divisor = svd_v_t[(3, 3)];
    if divisor.abs() < 1e-10 {
        return false;
    }

    *pt_world = (svd_v_t.column(3) / divisor).fixed_view::<3, 1>(0, 0).into_owned();

    // 检查解的质量
    if svd.singular_values.len() >= 4 && svd.singular_values[3] / svd.singular_values[2] < 1e-2 {
        // 解质量不好，放弃
        return false;
    }
    true
}

/// 将 nalgebra::Point2<f32> 转换为 Vector2<f64>
/// 
/// # 参数
/// - `p`：nalgebra::Point2<f32> 类型的点
/// 
/// # 返回值
/// 转换后的 Vector2<f64> 类型的点
pub fn to_vec2(p: Point2<f32>) -> Vector2<f64> {
    Vector2::new(p.x as f64, p.y as f64)
}

/// 单元测试模块
#[cfg(test)]
mod tests1 {
    // 引入外部模块中的函数和类型
    use super::*;
    use nalgebra::{Point2, Vector3};

    /// 测试 triangulation 函数
    #[test]
    fn test_triangulation() {
        // 创建示例位姿数组
        let mut poses = Vec::new();
        let rot = SO3::identity();
        let xyz = Vector3::zeros();
        let pose = SE3::from_rot_trans(rot, xyz);
        poses.push(pose);

        // 创建示例归一化平面上的点数组
        let points = vec![Vector3::zeros()];

        // 初始化世界坐标系中的三角测量点
        let mut pt_world = Vector3::zeros();

        // 调用 triangulation 函数进行测试
        let result = triangulation(&poses, &points, &mut pt_world);

        // 检查函数返回值是否为布尔类型
        assert!(result == true || result == false);
    }

    /// 测试 to_vec2 函数
    #[test]
    fn test_to_vec2() {
        // 创建示例 nalgebra::Point2<f32> 类型的点
        let p = Point2::new(1.0, 2.0);

        // 调用 to_vec2 函数进行转换
        let vec2 = to_vec2(p);

        // 检查转换后的向量的元素值是否正确
        assert!((vec2.x - 1.0 as f64) < 1e-6);
        assert!((vec2.y - 2.0 as f64) < 1e-6);
    }
}
