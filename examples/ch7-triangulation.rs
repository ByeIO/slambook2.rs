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
// #![feature(const_trait_impl)]
// #![feature(effects)]
// #![allow(incomplete_features)]
// #![feature(const_option)]

use image::{
    open, ImageBuffer, Rgb, DynamicImage
};
use imageproc::{
    drawing::draw_cross_mut, drawing::draw_line_segment_mut
};

use bye_orb_rs::{
    orb, fast, common::Matchable, orb::Brief
};

use nalgebra::{
    Matrix3, Vector3, Point2, Point3, U3, U4, Dim,
    Const, Matrix, ArrayStorage, Matrix4, Vector4
};

use std::time::Instant;

// 相机内参矩阵
const K : Matrix3<f32> = Matrix3::new(
    520.9, 0.0, 325.1,
    0.0, 521.0, 249.7,
    0.0, 0.0, 1.0
);

fn main() {
    let img1_path = "./assets/ch7-1.png";
    let img2_path = "./assets/ch7-2.png";

    main_triangulation(img1_path, img2_path);
}

// 寻找本质矩阵
fn find_essential_matrix(
    points1: &[Point2<f32>],
    points2: &[Point2<f32>]
) -> Matrix3<f32> {
    // 这里简化了本质矩阵的计算，实际应用中需要使用RANSAC等方法
    let focal_length = K[(0, 0)];
    let principal_point = Point2::new(K[(0, 2)], K[(1, 2)]);

    let normalized_points1: Vec<Point2<f32>> = points1.iter()
        .map(|p| Point2::new((p.x - principal_point.x) / focal_length, (p.y - principal_point.y) / focal_length))
        .collect();
    let normalized_points2: Vec<Point2<f32>> = points2.iter()
        .map(|p| Point2::new((p.x - principal_point.x) / focal_length, (p.y - principal_point.y) / focal_length))
        .collect();

    // 使用8点法计算本质矩阵
    let mut A : Matrix3<f32> = Matrix3::zeros();
    for i in 0..normalized_points1.len() {
        let x1 = normalized_points1[i].x;
        let y1 = normalized_points1[i].y;
        let x2 = normalized_points2[i].x;
        let y2 = normalized_points2[i].y;

        A[(0, 0)] += x1 * x2;
        A[(0, 1)] += x1 * y2;
        A[(0, 2)] += x1;
        A[(1, 0)] += y1 * x2;
        A[(1, 1)] += y1 * y2;
        A[(1, 2)] += y1;
        A[(2, 0)] += x2;
        A[(2, 1)] += y2;
        A[(2, 2)] += 1.0;
    }

    // 对A进行SVD分解
    let svd = A.svd(true, true);
    let U = svd.u.unwrap();
    let S = svd.singular_values;
    let V_t = svd.v_t.unwrap();

    // 构造本质矩阵
    let mut E : Matrix3<f32> = U * Matrix3::new(
        S[0], 0.0, 0.0,
        0.0, S[1], 0.0,
        0.0, 0.0, S[2]
    ) * V_t;

    // 强制本质矩阵的秩为2
    E[(2, 2)] = 0.0;

    // 返回值
    E
}

// 恢复位姿
fn recover_pose(
    E: &Matrix3<f32>,
    points1: &[Point2<f32>],
    points2: &[Point2<f32>]
) -> (Matrix3<f32>, Vector3<f32>) {
    // 对本质矩阵进行SVD分解
    let svd = E.svd(true, true);
    let U = svd.u.unwrap();
    let S = svd.singular_values;
    let V_t = svd.v_t.unwrap();

    // 构造旋转矩阵和平移向量
    let W = Matrix3::new(
        0.0, -1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 1.0
    );

    let R = U * W * V_t;
    let t = U.column(2).into_owned();

    (R, t)
}

fn main_triangulation(img1_path: &str, img2_path: &str) {
    // 读取图像
    let img1 = open(img1_path).unwrap();
    let img2 = open(img2_path).unwrap();

    // 找到特征匹配点
    let (keypoints1, keypoints2, matches) = find_feature_matches(&img1, &img2);

    // 估计两张图像间运动
    let (R, t) = pose_estimation_2d2d(&keypoints1, &keypoints2, &matches);

    // 三角化
    let points3d = triangulation(&keypoints1, &keypoints2, &matches, &R, &t);

    // 验证三角化点与特征点的重投影关系
    let mut img1_plot = img1.to_rgb8();
    let mut img2_plot = img2.to_rgb8();

    for (i, m) in matches.iter().enumerate() {
        // 第一个图
        let depth1 = points3d[i].z;
        println!("depth: {}", depth1);
        let pt1_cam = pixel2cam(Point2::new(keypoints1[m.0].x as f32, keypoints1[m.0].y as f32));
        let color = get_color(depth1);
        draw_cross_mut(&mut img1_plot, color, pt1_cam.x as i32, pt1_cam.y as i32);

        // 第二个图
        let pt2_trans = R * points3d[i] + t;
        let depth2 = pt2_trans.z;
        let color = get_color(depth2);
        draw_cross_mut(&mut img2_plot, color, keypoints2[m.1].x as i32, keypoints2[m.1].y as i32);
    }

    // 保存图像
    img1_plot.save("ch7-triangulation-img-1.png").unwrap();
    img2_plot.save("ch7-triangulation-img-2.png").unwrap();
}

fn find_feature_matches(img1: &DynamicImage, img2: &DynamicImage) -> (Vec<Brief>, Vec<Brief>, Vec<(usize, usize)>) {
    // 设置关键点数量
    let n_keypoints = 500;

    // 检测关键点并计算描述子
    let keypoints1 = orb::orb(&mut img1.clone(), n_keypoints).unwrap();
    let keypoints2 = orb::orb(&mut img2.clone(), n_keypoints).unwrap();

    // 使用Hamming距离进行匹配
    let pair_indices = orb::match_brief(&keypoints1, &keypoints2);

    // 筛选匹配点
    let mut matches: Vec<(usize, usize, f32)> = pair_indices.iter()
        .map(|&(i, j)| (i, j, keypoints1[i].distance(&keypoints2[j]) as f32))
        .collect();

    matches.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

    let min_dist = matches[0].2;
    let max_dist = matches[matches.len() - 1].2;

    println!("-- Max dist: {}", max_dist);
    println!("-- Min dist: {}", min_dist);

    // 筛选出距离小于两倍最小距离的匹配点
    let good_matches: Vec<(usize, usize)> = matches.iter()
        .filter(|&&(_, _, dist)| dist <= (2.0 * min_dist).max(30.0))
        .map(|&(i, j, _)| (i, j))
        .collect();

    println!("一共找到了 {} 组匹配点", good_matches.len());

    (keypoints1, keypoints2, good_matches)
}

fn pose_estimation_2d2d(
    keypoints1: &[Brief],
    keypoints2: &[Brief],
    matches: &[(usize, usize)],
) -> (Matrix3<f32>, Vector3<f32>) {

    // 将匹配点转换为Point2f形式
    let points1: Vec<Point2<f32>> = matches.iter()
        .map(|&(i, _)| Point2::new(keypoints1[i].x as f32, keypoints1[i].y as f32))
        .collect();

    let points2: Vec<Point2<f32>> = matches.iter()
        .map(|&(_, j)| Point2::new(keypoints2[j].x as f32, keypoints2[j].y as f32))
        .collect();

    // 计算本质矩阵
    let essential_matrix = find_essential_matrix(&points1, &points2);

    // 从本质矩阵中恢复旋转和平移信息
    let (R, t) = recover_pose(&essential_matrix, &points1, &points2);

    (R, t)
}

// 三角测量
fn triangulation(
    keypoints1: &[Brief],
    keypoints2: &[Brief],
    matches: &[(usize, usize)],
    R: &Matrix3<f32>,
    t: &Vector3<f32>
) -> Vec<Point3<f32>> {

    // 构造投影矩阵 P1 和 P2, 3x4 的矩阵
    let mut P1 = Matrix::<f32, Const<3>, Const<4>, ArrayStorage<f32, 3, 4>>::zeros();
    let mut P2 = Matrix::<f32, Const<3>, Const<4>, ArrayStorage<f32, 3, 4>>::zeros();

    // P1 是 [I | 0]
    P1.fixed_view_mut::<3, 3>(0, 0).copy_from(&Matrix3::identity());
    P1.fixed_view_mut::<3, 1>(0, 3).fill(0.0);

    // P2 是 [R | t]
    P2.fixed_view_mut::<3, 3>(0, 0).copy_from(R);
    P2.fixed_view_mut::<3, 1>(0, 3).copy_from(t);

    // 将像素坐标转换为相机归一化坐标
    let pts1: Vec<Point2<f32>> = matches.iter()
        .map(|&(i, _)| pixel2cam(Point2::new(keypoints1[i].x as f32, keypoints1[i].y as f32)))
        .collect();

    let pts2: Vec<Point2<f32>> = matches.iter()
        .map(|&(_, j)| pixel2cam(Point2::new(keypoints2[j].x as f32, keypoints2[j].y as f32)))
        .collect();

    // 三角化
    let pts4d_hom = triangulate_points(&P1, &P2, &pts1, &pts2);
    let pts4d: Vec<Point3<f32>> = pts4d_hom.iter()
        .map(|p| Point3::new(p.x / p.w, p.y / p.w, p.z / p.w))
        .collect();

    pts4d
}

fn triangulate_points(
    P1: &Matrix<f32, Const<3>, Const<4>, ArrayStorage<f32, 3, 4>>,
    P2: &Matrix<f32, Const<3>, Const<4>, ArrayStorage<f32, 3, 4>>,
    pts1: &[Point2<f32>],
    pts2: &[Point2<f32>],
) -> Vec<Vector4<f32>> {
    let mut pts4d = Vec::new();
    for i in 0..pts1.len() {
        let A = Matrix4::new(
            pts1[i].x * P1[(2, 0)] - P1[(0, 0)],
            pts1[i].x * P1[(2, 1)] - P1[(0, 1)],
            pts1[i].x * P1[(2, 2)] - P1[(0, 2)],
            pts1[i].x * P1[(2, 3)] - P1[(0, 3)],
            pts1[i].y * P1[(2, 0)] - P1[(1, 0)],
            pts1[i].y * P1[(2, 1)] - P1[(1, 1)],
            pts1[i].y * P1[(2, 2)] - P1[(1, 2)],
            pts1[i].y * P1[(2, 3)] - P1[(1, 3)],
            pts2[i].x * P2[(2, 0)] - P2[(0, 0)],
            pts2[i].x * P2[(2, 1)] - P2[(0, 1)],
            pts2[i].x * P2[(2, 2)] - P2[(0, 2)],
            pts2[i].x * P2[(2, 3)] - P2[(0, 3)],
            pts2[i].y * P2[(2, 0)] - P2[(1, 0)],
            pts2[i].y * P2[(2, 1)] - P2[(1, 1)],
            pts2[i].y * P2[(2, 2)] - P2[(1, 2)],
            pts2[i].y * P2[(2, 3)] - P2[(1, 3)],
        );

        let svd = A.svd(true, true);
        let V_t = svd.v_t.unwrap();
        let pt4d = V_t.column(3).into_owned();
        pts4d.push(pt4d);
    }

    pts4d
}

fn pixel2cam(p: Point2<f32>) -> Point2<f32> {
    Point2::new(
        (p.x - K[(0, 2)]) / K[(0, 0)],
        (p.y - K[(1, 2)]) / K[(1, 1)]
    )
}

fn get_color(depth: f32) -> Rgb<u8> {
    let up_th = 50.0;
    let low_th = 10.0;
    let th_range = up_th - low_th;
    let depth = depth.max(low_th).min(up_th);
    let r = (255.0 * depth / th_range) as u8;
    let b = (255.0 * (1.0 - depth / th_range)) as u8;
    Rgb([r, 0, b])
}