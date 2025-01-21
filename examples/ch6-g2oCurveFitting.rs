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

// 线性代数
use nalgebra::{DMatrix, DVector, Matrix3, Vector3, SVector, Matrix, VecStorage, Dyn};

// 图优化
use factrs::{
    assign_symbols,
    core::{BetweenResidual, GaussNewton, Graph, Values},
    dtype, fac,
    linalg::{Const, ForwardProp, Numeric, NumericalDiff, VectorX, DiffResult, MatrixX},
    residuals::Residual1,
    traits::*,
    variables::{VectorVar2, SE2, VectorVar3},
    containers::Key,
};

use rand::{Rng, SeedableRng};
use rand::distributions::{Distribution, Uniform};

use std::process::exit;
use std::ops::Add;
use std::ops::Mul;

// 曲线拟合的因子
#[derive(Clone, Debug)]
pub struct CurveFittingFactor {
    x: f64,
    measurement: f64,
}

impl CurveFittingFactor {
    pub fn new(x: f64, measurement: f64) -> Self {
        Self { x, measurement }
    }
}

// 实现 Residual1 trait 用于因子
// `mark` 宏处理序列化内容以及一些自定义实现
#[factrs::mark]
impl Residual1 for CurveFittingFactor {
    type Differ = ForwardProp<<Self as Residual1>::DimIn>;
    type V1 = VectorVar3;
    type DimIn = Const<3>;
    type DimOut = Const<1>;

    // residual1_jacobian要放在residual1之前
    // fn residual1_jacobian(&self, values: &Values, keys: &[Key]) -> DiffResult<VectorX, MatrixX> {
    //     let abc: &VectorVar2 = values
    //         .get_unchecked(keys[0])
    //         .expect("got wrong variable type");
    //     let y = (abc[0] * self.x * self.x + abc[1] * self.x + abc[2]).exp();
    //     let jacobian = MatrixX::from_row_slice(1, 3, &[-self.x * self.x * y, -self.x * y, -y]);
    //     DiffResult {
    //         value: self.residual1(abc.clone()),
    //         diff: jacobian,
    //     }
    // }

    fn residual1<T: Numeric>(&self, v: VectorVar3<T>) -> VectorX<T> {
        let abc = v.to_owned();
        let x_squared = T::from_f64(self.x * self.x).unwrap_or_else(|| T::from_f64(0.0).unwrap());
        let x = T::from_f64(self.x).unwrap_or_else(|| T::from_f64(0.0).unwrap());
        let measurement = T::from_f64(self.measurement).unwrap_or_else(|| T::from_f64(0.0).unwrap());
        println!("abc: {:#?}", abc);
        let error = measurement - (abc[0] * x_squared + abc[1] * x + abc[2]).exp();
        // 返回值
        VectorX::from_column_slice(&[error])
    }

}

// 定义符号变量
assign_symbols!(X: VectorVar3);

fn main() {
    // 真实参数值
    let ar = 1.0;
    let br = 2.0;
    let cr = 1.0;
    // 估计参数值
    let ae = 2.0;
    let be = -1.0;
    let ce = 5.0;
    // 数据点
    let N = 100;
    // 噪声Sigma值
    let w_sigma = 1.0;
    // 计算精度
    let inv_sigma = 1.0 / w_sigma;

    // 随机数产生器
    let mut rng = rand::rngs::StdRng::seed_from_u64(42);
    let distribution = Uniform::new(-1.0, 1.0);

    let mut x_data = Vec::new();
    let mut y_data = Vec::new();

    for i in 0..N {
        let x = i as f64 / 100.0;
        x_data.push(x);
        y_data.push((ar * x * x + br * x + cr).exp() + rng.sample(distribution) * w_sigma * w_sigma);
    }

    // 构建图优化
    let mut graph = Graph::new();

    // 添加因子
    for i in 0..N {
        let factor = CurveFittingFactor::new(x_data[i], y_data[i]);
        let factor_node = fac![factor, X(0), inv_sigma as std];
        graph.add_factor(factor_node);
    }

    // 创建初始值
    let mut values = Values::new();
    values.insert(X(0), VectorVar3::new(ae, be, ce));

    // 优化
    let mut opt: GaussNewton = GaussNewton::new(graph);
    let result = opt.optimize(values).expect("优化失败");

    println!("最终结果: {:#?}", result);
}