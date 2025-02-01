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

//! 

use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::fs::create_dir_all;

use std::cell::{
    RefMut, Ref, RefCell
};

// 线性代数
use nalgebra::{
    Quaternion, Vector3, Matrix3, UnitQuaternion, 
    Isometry3, Translation3, Const,
    Matrix, Point3, ViewStorage, Rotation3,
    Matrix3x1, VectorView3, DMatrix, DVector, SVector,
    Vector6, Matrix6, Vector2, Matrix2, 
};

// 李代数
use factrs::{
    assign_symbols,
    core::{BetweenResidual, GaussNewton, Graph, Values},
    dtype, fac,
    linalg::{ ForwardProp, Numeric, NumericalDiff, VectorX, DiffResult, MatrixX },
    residuals::{Residual1, Residual2},
    traits::*,
    variables::{VectorVar2, SE2, VectorVar3, SE3, SO3, SO2, MatrixLieGroup},
    containers::Key,
    noise::{GaussianNoise},
    optimizers::{LevenMarquardt}
};

// 图像处理
use image::{
    DynamicImage, GrayImage, ImageBuffer, Luma, 
    GenericImageView, Rgb, RgbImage, 
    buffer::ConvertBuffer,
};
use imageproc::drawing::{
    draw_cross_mut, draw_filled_circle_mut, draw_line_segment_mut,
};

// 随机数
use rand::Rng;
use rand_distr::{
    Distribution, Normal
};
