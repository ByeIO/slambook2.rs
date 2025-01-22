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

use image::{
    open, ImageBuffer, Rgb, DynamicImage
};
use imageproc::{
    drawing::draw_cross_mut, drawing::draw_line_segment_mut
};

use nalgebra::{
    Matrix3, Vector3, Point2, Point3
};

use bye_orb_rs::{
    orb, fast, common::Matchable, orb::Brief
};

use std::time::Instant;

fn main() {
    let img1_path = "./assets/ch7-1.png";
    let img2_path = "./assets/ch7-2.png";

    main_orb(img1_path, img2_path);
}

fn main_orb(img1_path: &str, img2_path: &str) {
    // 读取图像
    let mut img1 = open(img1_path).unwrap();
    let mut img2 = open(img2_path).unwrap();

    // 设置关键点数量
    let n_keypoints = 500;

    // 第一步: 检测Oriented FAST角点位置并计算BRIEF描述子
    let start_time = Instant::now();
    let img1_keypoints = orb::orb(&mut img1, n_keypoints).unwrap();
    let img2_keypoints = orb::orb(&mut img2, n_keypoints).unwrap();
    let end_time = Instant::now();
    println!("提取ORB特征点耗时: {:?} 秒", end_time - start_time);

    // 第二步: 使用Hamming距离进行匹配
    let start_time = Instant::now();
    let pair_indices = orb::match_brief(&img1_keypoints, &img2_keypoints);
    let end_time = Instant::now();
    println!("匹配ORB特征点耗时: {:?} 秒", end_time - start_time);

    // 第三步: 匹配点对筛选
    let mut matches: Vec<(usize, usize, f32)> = pair_indices.iter()
        .map(|&(i, j)| (i, j, img1_keypoints[i].distance(&img2_keypoints[j]) as f32))
        .collect();

    matches.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

    let min_dist = matches[0].2;
    let max_dist = matches[matches.len() - 1].2;

    println!("-- 最大距离: {}", max_dist);
    println!("-- 最小距离: {}", min_dist);

    // 当描述子之间的距离大于两倍的最小距离时，认为匹配有误。但最小距离可能会非常小，设置一个经验值30作为下限。
    let good_matches: Vec<(usize, usize, f32)> = matches.iter()
        .filter(|&&(_, _, dist)| dist <= (2.0 * min_dist).max(30.0))
        .cloned()
        .collect();

    // 第四步: 绘制匹配结果
    let mut img1_rgb = img1.to_rgb8();
    let mut img2_rgb = img2.to_rgb8();

    // 绘制所有匹配点
    for &(i, j, _) in &matches {
        let kp1 = (img1_keypoints[i].x, img1_keypoints[i].y);
        let kp2 = (img2_keypoints[j].x, img2_keypoints[j].y);
        let color = Rgb([0, 255, 0]);
        draw_cross_mut(&mut img1_rgb, color, kp1.0 as i32, kp1.1 as i32);
        draw_cross_mut(&mut img2_rgb, color, kp2.0 as i32, kp2.1 as i32);
    }

    // 保存绘制结果
    img1_rgb.save("ch7-pose_estimation_2d2d-all_matches.png").unwrap();
    img2_rgb.save("ch7-pose_estimation_2d2d-all_matches2.png").unwrap();

    // 绘制筛选后的匹配点
    let mut img1_rgb_good = img1.to_rgb8();
    let mut img2_rgb_good = img2.to_rgb8();

    for &(i, j, _) in &good_matches {
        let kp1 = (img1_keypoints[i].x, img1_keypoints[i].y);
        let kp2 = (img2_keypoints[j].x, img2_keypoints[j].y);
        let color = Rgb([0, 255, 0]);
        draw_cross_mut(&mut img1_rgb_good, color, kp1.0 as i32, kp1.1 as i32);
        draw_cross_mut(&mut img2_rgb_good, color, kp2.0 as i32, kp2.1 as i32);
    }

    // 保存绘制结果
    img1_rgb_good.save("ch7-pose_estimation_2d2d-good_matches.png").unwrap();
    img2_rgb_good.save("ch7-pose_estimation_2d2d-good_matches2.png").unwrap();

    // 第五步: 创建并排放置的图像
    let (width1, height1) = img1_rgb.dimensions();
    let (width2, height2) = img2_rgb.dimensions();
    let total_width = width1 + width2;
    let max_height = height1.max(height2);

    let mut combined_image = ImageBuffer::new(total_width, max_height);

    // 将第一张图片复制到组合图像的左侧
    for y in 0..height1 {
        for x in 0..width1 {
            combined_image.put_pixel(x, y, *img1_rgb.get_pixel(x, y));
        }
    }

    // 将第二张图片复制到组合图像的右侧
    for y in 0..height2 {
        for x in 0..width2 {
            combined_image.put_pixel(width1 + x, y, *img2_rgb.get_pixel(x, y));
        }
    }

    // 绘制连接线
    for &(i, j, _) in &good_matches {
        let kp1 = (img1_keypoints[i].x, img1_keypoints[i].y);
        let kp2 = (img2_keypoints[j].x, img2_keypoints[j].y);
        let color = Rgb([0, 255, 0]);
        draw_line_segment_mut(
            &mut combined_image,
            (kp1.0 as f32, kp1.1 as f32),
            ((width1 as f32 + kp2.0 as f32), kp2.1 as f32),
            color,
        );
    }

    // 保存并排放置的图像
    combined_image.save("ch7-pose_estimation_2d2d-combined_matches.png").unwrap();

    // 第六步: 2D-2D姿态估计
    let K = Matrix3::new(
        520.9, 0.0, 325.1,
        0.0, 521.0, 249.7,
        0.0, 0.0, 1.0
    );

    let (R, t) = pose_estimation_2d2d(&img1_keypoints, &img2_keypoints, &good_matches, &K);

    println!("Rotation matrix R is:");
    println!("{}", R);
    println!("Translation vector t is:");
    println!("{}", t);

    // 验证对极约束
    for &(i, j, _) in &good_matches {
        let pt1 = pixel2cam(Point2::new(img1_keypoints[i].x as f32, img1_keypoints[i].y as f32), &K);
        let pt2 = pixel2cam(Point2::new(img2_keypoints[j].x as f32, img2_keypoints[j].y as f32), &K);

        let y1 = Vector3::new(pt1.x, pt1.y, 1.0);
        let y2 = Vector3::new(pt2.x, pt2.y, 1.0);

        // 创建反对称矩阵 t_x
        let t_x = Matrix3::new(
            0.0, -t.z, t.y,
            t.z, 0.0, -t.x,
            -t.y, t.x, 0.0
        );

        // 计算对极约束
        let d = y2.transpose() * t_x * R * y1;
        println!("Epipolar constraint = {}", d);
    }
}

fn pixel2cam(p: Point2<f32>, K: &Matrix3<f32>) -> Point2<f32> {
    Point2::new(
        (p.x - K[(0, 2)]) / K[(0, 0)],
        (p.y - K[(1, 2)]) / K[(1, 1)]
    )
}

fn pose_estimation_2d2d(
    keypoints1: &[Brief],
    keypoints2: &[Brief],
    matches: &[(usize, usize, f32)],
    K: &Matrix3<f32>
) -> (Matrix3<f32>, Vector3<f32>) {
    // 创建匹配点对的向量
    let points1: Vec<Point2<f32>> = matches.iter()
        .map(|&(i, _, _)| Point2::new(keypoints1[i].x as f32, keypoints1[i].y as f32))
        .collect();

    let points2: Vec<Point2<f32>> = matches.iter()
        .map(|&(_, j, _)| Point2::new(keypoints2[j].x as f32, keypoints2[j].y as f32))
        .collect();

    // 计算本质矩阵
    let essential_matrix = find_essential_matrix(&points1, &points2, K);

    // 从本质矩阵中恢复旋转和平移信息
    let (R, t) = recover_pose(&essential_matrix, &points1, &points2, K);

    (R, t)
}

fn find_essential_matrix(
    points1: &[Point2<f32>],
    points2: &[Point2<f32>],
    K: &Matrix3<f32>
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

fn recover_pose(
    E: &Matrix3<f32>,
    points1: &[Point2<f32>],
    points2: &[Point2<f32>],
    K: &Matrix3<f32>
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