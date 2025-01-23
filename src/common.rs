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

// 线性代数
use nalgebra::{
    DMatrix, DVector, Matrix3, Vector3, 
    Vector2, Matrix6, Vector6, Matrix2x6, 
    SMatrix, SVector, Matrix2
};

// 图像处理
use image::{
    open, ImageBuffer, Rgb, DynamicImage, Luma, RgbImage,
    buffer::ConvertBuffer
};
use imageproc::{
    drawing::draw_cross_mut, drawing::draw_line_segment_mut
};

// ORB角点检测
use bye_orb_rs::{
    orb, fast, common::Matchable
};

// 图优化
use factrs::{
    assign_symbols,
    core::{BetweenResidual, GaussNewton, Graph, Values},
    dtype, fac,
    linalg::{Const, ForwardProp, Numeric, NumericalDiff, VectorX, DiffResult, MatrixX},
    residuals::Residual1,
    traits::*,
    variables::{VectorVar2, SE2, VectorVar3, SE3, SO2, SO3, MatrixLieGroup},
    containers::Key,
    noise::{GaussianNoise}
};

// 随机数
use rand::{
    Rng, SeedableRng
};
use rand::distributions::{
    Distribution, Uniform
};

// 标准库
use std::time::Instant;
use std::cell::RefCell;
use std::f64::consts::PI;
use std::f64::EPSILON;
use std::ops::{Add, Div, Mul, Neg, Sub};
use std::cmp::PartialOrd;

// 定义符号变量
assign_symbols!(_X_: VectorVar3);
assign_symbols!(_Y_: SE2);
assign_symbols!(_Z_: SE3);

// 双线性插值
pub fn get_pixel_value(img: &ImageBuffer<Luma<u8>, Vec<u8>>, x: f64, y: f64) -> f64 {
    let mut x = x;
    let mut y = y;
    
    // 边界检查
    if x < 0.0 {
        x = 0.0;
    }
    if y < 0.0 {
        y = 0.0;
    }
    if x >= img.width() as f64 - 1.0 {
        x = img.width() as f64 - 1.0;
    }
    if y >= img.height() as f64 - 1.0 {
        y = img.height() as f64 - 1.0;
    }

    let x_floor = x.floor() as u32;
    let y_floor = y.floor() as u32;
    let xx = x - x_floor as f64;
    let yy = y - y_floor as f64;

    // 确保不越界
    let x_ceil = (x_floor + 1).min(img.width() - 1);
    let y_ceil = (y_floor + 1).min(img.height() - 1);

    let data = |x: u32, y: u32| img.get_pixel(x, y)[0] as f64;

    (1.0 - xx) * (1.0 - yy) * data(x_floor, y_floor) +
    xx * (1.0 - yy) * data(x_ceil, y_floor) +
    (1.0 - xx) * yy * data(x_floor, y_ceil) +
    xx * yy * data(x_ceil, y_ceil)
}

/* start 随机数相关 */
// 生成 [0, 1) 范围内的随机浮点数
pub fn rand_double() -> f64 {
    let mut rng = rand::thread_rng();
    rng.gen::<f64>()
}

// 使用 Box-Muller 变换生成标准正态分布的随机数
pub fn rand_normal() -> f64 {
    let mut u1;
    let mut u2;
    let mut w;

    loop {
        // 生成两个均匀分布的随机数
        u1 = 2.0 * rand_double() - 1.0; // 映射到 [-1, 1)
        u2 = 2.0 * rand_double() - 1.0; // 映射到 [-1, 1)
        w = u1 * u1 + u2 * u2; // 计算半径平方

        // 确保 w 在 (0, 1) 范围内
        if w < 1.0 && w != 0.0 {
            break;
        }
    }

    // Box-Muller 变换
    let factor = (-2.0 * w.ln() / w).sqrt();
    u1 * factor
}

/* end 随机数相关 */

/* start 旋转矩阵相关 */

// 用于旋转转换的数学函数。
// 点积和叉积

pub fn dot_product<T>(x: &[T; 3], y: &[T; 3]) -> T
where
    T: Mul<Output = T> + Add<Output = T> + Copy,
{
    x[0] * y[0] + x[1] * y[1] + x[2] * y[2]
}

pub fn cross_product<T>(x: &[T; 3], y: &[T; 3], result: &mut [T; 3])
where
    T: Mul<Output = T> + Sub<Output = T> + Copy,
{
    result[0] = x[1] * y[2] - x[2] * y[1];
    result[1] = x[2] * y[0] - x[0] * y[2];
    result[2] = x[0] * y[1] - x[1] * y[0];
}

// 将角轴转换为四元数
pub fn angle_axis_to_quaternion<T>(angle_axis: &[T; 3], quaternion: &mut [T; 4])
where
    T: Mul<Output = T>
        + Add<Output = T>
        + Div<Output = T>
        + Sub<Output = T>
        + Copy
        + From<f64>
        + PartialOrd
        + Neg<Output = T>,
    f64: From<T>
{
    let a0 = angle_axis[0];
    let a1 = angle_axis[1];
    let a2 = angle_axis[2];
    let theta_squared = a0 * a0 + a1 * a1 + a2 * a2;

    if theta_squared > T::from(EPSILON) {
        let theta = sqrt(theta_squared);
        let half_theta = theta * T::from(0.5);
        let k = sin(half_theta) / theta;
        quaternion[0] = cos(half_theta);
        quaternion[1] = a0 * k;
        quaternion[2] = a1 * k;
        quaternion[3] = a2 * k;
    } else {
        // 如果theta_squared为零
        let k = T::from(0.5);
        quaternion[0] = T::from(1.0);
        quaternion[1] = a0 * k;
        quaternion[2] = a1 * k;
        quaternion[3] = a2 * k;
    }
}

// 将四元数转换为角轴
pub fn quaternion_to_angle_axis<T>(quaternion: &[T; 4], angle_axis: &mut [T; 3])
where
    T: Mul<Output = T>
        + Add<Output = T>
        + Div<Output = T>
        + Sub<Output = T>
        + Copy
        + From<f64>
        + PartialOrd
        + Neg<Output = T>,
    f64: From<T>
{
    let q1 = quaternion[1];
    let q2 = quaternion[2];
    let q3 = quaternion[3];
    let sin_squared_theta = q1 * q1 + q2 * q2 + q3 * q3;

    // 对于表示非零旋转的四元数，转换是数值稳定的
    if sin_squared_theta > T::from(EPSILON) {
        let sin_theta = sqrt(sin_squared_theta);
        let cos_theta = quaternion[0];

        // 如果cos_theta为负，theta大于pi/2，这意味着角轴向量的角度将大于pi...
        let two_theta = T::from(2.0)
            * (if cos_theta < T::from(0.0) {
                atan2(-sin_theta, -cos_theta)
            } else {
                atan2(sin_theta, cos_theta)
            });
        let k = two_theta / sin_theta;

        angle_axis[0] = q1 * k;
        angle_axis[1] = q2 * k;
        angle_axis[2] = q3 * k;
    } else {
        // 对于零旋转，sqrt()会产生NaN，因为参数为零。通过使用泰勒级数近似，
        // 并在第一项截断，当使用Jets时，值和一阶导数将被正确计算。
        let k = T::from(2.0);
        angle_axis[0] = q1 * k;
        angle_axis[1] = q2 * k;
        angle_axis[2] = q3 * k;
    }
}

// 使用角轴旋转点
pub fn angle_axis_rotate_point<T>(angle_axis: &[T; 3], pt: &[T; 3], result: &mut [T; 3])
where
    T: Mul<Output = T>
        + Add<Output = T>
        + Div<Output = T>
        + Sub<Output = T>
        + Copy
        + From<f64>
        + PartialOrd
        + Neg<Output = T>,
    f64: From<T>
{
    let theta2 = dot_product(angle_axis, angle_axis);
    if theta2 > T::from(EPSILON) {
        // 远离零时，使用罗德里格斯公式
        //
        //   result = pt * costheta +
        //            (w x pt) * sintheta +
        //            w (w . pt) (1 - costheta)
        //
        // 我们要小心，只有在角轴向量的范数大于零时才计算平方根。否则会得到除以零的错误。
        //
        let theta = sqrt(theta2);
        let costheta = cos(theta);
        let sintheta = sin(theta);
        let theta_inverse = T::from(1.0) / theta;

        let w = [
            angle_axis[0] * theta_inverse,
            angle_axis[1] * theta_inverse,
            angle_axis[2] * theta_inverse,
        ];

        let mut w_cross_pt = [T::from(0.0); 3];
        cross_product(&w, pt, &mut w_cross_pt);

        let tmp = dot_product(&w, pt) * (T::from(1.0) - costheta);

        result[0] = pt[0] * costheta + w_cross_pt[0] * sintheta + w[0] * tmp;
        result[1] = pt[1] * costheta + w_cross_pt[1] * sintheta + w[1] * tmp;
        result[2] = pt[2] * costheta + w_cross_pt[2] * sintheta + w[2] * tmp;
    } else {
        // 接近零时，旋转矩阵R的一阶泰勒近似为
        //
        //   R = I + hat(w) * sin(theta)
        //
        // 但sin(theta) ~ theta，且theta * w = angle_axis，这给了我们
        //
        //  R = I + hat(w)
        //
        // 实际上与点pt相乘，得到 R * pt = pt + w x pt。
        //
        // 在接近零时切换到泰勒展开，使用Jets评估时提供了有意义的一阶导数。
        //
        let mut w_cross_pt = [T::from(0.0); 3];
        cross_product(angle_axis, pt, &mut w_cross_pt);

        result[0] = pt[0] + w_cross_pt[0];
        result[1] = pt[1] + w_cross_pt[1];
        result[2] = pt[2] + w_cross_pt[2];
    }
}

// 手动实现 sqrt 函数
pub fn sqrt<T>(x: T) -> T
where
    T: From<f64> + Into<f64> + Copy,
{
    T::from(x.into().sqrt())
}

// 手动实现 sin 函数
pub fn sin<T>(x: T) -> T
where
    T: From<f64> + Into<f64> + Copy,
{
    T::from(x.into().sin())
}

// 手动实现 cos 函数
pub fn cos<T>(x: T) -> T
where
    T: From<f64> + Into<f64> + Copy,
{
    T::from(x.into().cos())
}

// 手动实现 atan2 函数
pub fn atan2<T>(y: T, x: T) -> T
where
    T: From<f64> + Into<f64> + Copy,
{
    T::from(y.into().atan2(x.into()))
}

/* end 旋转矩阵相关 */