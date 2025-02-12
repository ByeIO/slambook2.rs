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

//! 公共导入

// 标准库
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Condvar, Mutex,
};
use std::thread;
use std::time::Duration;

// 线性代数
pub use nalgebra::{
    DMatrix, DVector, Matrix2, Matrix2x3, Matrix2x4, Matrix2x6,  Matrix3, Matrix3x2, Matrix3x4, Matrix3x6, Matrix4, Matrix4x2, Matrix4x3, Matrix4x6, Matrix5, Matrix5x3, Matrix6, Matrix6x2, Matrix6x3, Matrix6x4, Vector2, Vector3, Vector4, Vector5, Vector6,
    Const, Matrix, Vector, SMatrix, SVector, OMatrix, OVector, Dyn, Quaternion, UnitQuaternion, 
    Isometry3, Translation3, ViewStorage, Rotation3,
    VectorView, 
};

// 双精度矩阵
pub type MatXX = OMatrix<f64, Dyn, Dyn>;
pub type Mat1010 = SMatrix<f64, 10, 10>;
pub type Mat1313 = SMatrix<f64, 13, 13>;
pub type Mat810 = SMatrix<f64, 8, 10>;
pub type Mat83 = SMatrix<f64, 8, 3>;
pub type Mat66 = SMatrix<f64, 6, 6>;
pub type Mat53 = SMatrix<f64, 5, 3>;
pub type Mat43 = SMatrix<f64, 4, 3>;
pub type Mat42 = SMatrix<f64, 4, 2>;
pub type Mat33 = SMatrix<f64, 3, 3>;
pub type Mat22 = SMatrix<f64, 2, 2>;
pub type Mat88 = SMatrix<f64, 8, 8>;
pub type Mat77 = SMatrix<f64, 7, 7>;
pub type Mat49 = SMatrix<f64, 4, 9>;
pub type Mat89 = SMatrix<f64, 8, 9>;
pub type Mat94 = SMatrix<f64, 9, 4>;
pub type Mat98 = SMatrix<f64, 9, 8>;
pub type Mat81 = SVector<f64, 8>;
pub type Mat18 = SMatrix<f64, 1, 8>;
pub type Mat91 = SVector<f64, 9>;
pub type Mat19 = SMatrix<f64, 1, 9>;
pub type Mat84 = SMatrix<f64, 8, 4>;
pub type Mat48 = SMatrix<f64, 4, 8>;
pub type Mat44 = SMatrix<f64, 4, 4>;
pub type Mat34 = SMatrix<f64, 3, 4>;
pub type Mat1414 = SMatrix<f64, 14, 14>;

// 单精度矩阵
pub type Mat33f = SMatrix<f32, 3, 3>;
pub type Mat103f = SMatrix<f32, 10, 3>;
pub type Mat22f = SMatrix<f32, 2, 2>;
pub type Vec3f = SVector<f32, 3>;
pub type Vec2f = SVector<f32, 2>;
pub type Vec6f = SVector<f32, 6>;
pub type Mat18f = SMatrix<f32, 1, 8>;
pub type Mat66f = SMatrix<f32, 6, 6>;
pub type Mat88f = SMatrix<f32, 8, 8>;
pub type Mat84f = SMatrix<f32, 8, 4>;
pub type Mat44f = SMatrix<f32, 4, 4>;
pub type Mat1212f = SMatrix<f32, 12, 12>;
pub type Mat1313f = SMatrix<f32, 13, 13>;
pub type Mat1010f = SMatrix<f32, 10, 10>;
pub type Mat99f = SMatrix<f32, 9, 9>;
pub type Mat42f = SMatrix<f32, 4, 2>;
pub type Mat62f = SMatrix<f32, 6, 2>;
pub type Mat12f = SMatrix<f32, 1, 2>;
pub type MatXXf = OMatrix<f32, Dyn, Dyn>;
pub type Mat1414f = SMatrix<f32, 14, 14>;

// 双精度向量
pub type Vec14 = SVector<f64, 14>;
pub type Vec13 = SVector<f64, 13>;
pub type Vec10 = SVector<f64, 10>;
pub type Vec9 = SVector<f64, 9>;
pub type Vec8 = SVector<f64, 8>;
pub type Vec7 = SVector<f64, 7>;
pub type Vec6 = SVector<f64, 6>;
pub type Vec5 = SVector<f64, 5>;
pub type Vec4 = SVector<f64, 4>;
pub type Vec3 = SVector<f64, 3>;
pub type Vec2 = SVector<f64, 2>;
pub type VecX = OVector<f64, Dyn>;

// 单精度向量
pub type Vec12f = SVector<f32, 12>;
pub type Vec8f = SVector<f32, 8>;
pub type Vec10f = SVector<f32, 10>;
pub type Vec4f = SVector<f32, 4>;
pub type Vec13f = SVector<f32, 13>;
pub type Vec9f = SVector<f32, 9>;
pub type VecXf = OVector<f32, Dyn>;
pub type Vec14f = SVector<f32, 14>;

// 李代数
pub use factrs::{
    assign_symbols,
    core::{
        BetweenResidual, GaussNewton, Graph, Values,
    },
    dtype, fac,
    linalg::{ 
        ForwardProp, Numeric, NumericalDiff, VectorX, DiffResult, MatrixX 
    },
    residuals::Residual1,
    traits::*,
    variables::{
        VectorVar2, VectorVar3,SE2, SE3, SO2, SO3, MatrixLieGroup,
    },
    containers::Key,
    noise::{
        GaussianNoise, UnitNoise,
    },
};

// 计算机视觉
pub use cv_core;
pub use image;
pub use imageproc;

// 日志
pub use env_logger;
pub use log;
