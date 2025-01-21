#![allow(dead_code)] // 允许未使用的代码
#![allow(unused_variables)] // 允许未使用的变量
#![allow(unused_imports)] // 允许未使用的导入
#![allow(unused_mut)] // 允许未使用的可变变量
#![allow(unused_assignments)] // 允许未使用的赋值
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // 在非调试模式下设置Windows子系统
#![allow(rustdoc::missing_crate_level_docs)] // 允许缺少crate级别的文档
#![allow(unsafe_code)] // 允许使用unsafe代码
#![allow(clippy::undocumented_unsafe_blocks)] // 允许未文档化的unsafe块
#![allow(unused_must_use)] // 允许未使用的must_use结果
#![allow(non_snake_case)] // 允许非蛇形命名

// 使用fastrand库生成随机数
use fastrand::Rng; 
// 使用gomez库中的nalgebra模块
use gomez::nalgebra as na; 
// 使用gomez库中的优化相关模块
use gomez::{
    Domain, Function, Optimizer, OptimizerDriver, 
    Problem, Sample
}; 
// 使用nalgebra库中的存储和向量相关模块
use na::{
    storage::StorageMut, Dyn, IsContiguous, Vector
}; 

// 定义一个随机优化器
struct Random {
    // 随机数生成器
    rng: Rng, 
}

impl Random {
    fn new(rng: Rng) -> Self {
        Self { rng }
    }
}

// 实现Optimizer trait，用于随机优化
impl<F: Function> Optimizer<F> for Random
where
    F::Field: Sample, // 要求F的字段类型实现Sample trait
{
    // 优化器名称
    const NAME: &'static str = "Random"; 
    // 错误类型为不可恢复错误
    type Error = std::convert::Infallible; 

    // 优化器的下一步操作
    fn opt_next<Sx>(
        &mut self,
        f: &F,
        dom: &Domain<F::Field>,
        x: &mut Vector<F::Field, Dyn, Sx>,
    ) -> Result<F::Field, Self::Error>
    where
        Sx: StorageMut<F::Field, Dyn> + IsContiguous, // 要求存储类型是可变的且连续的
    {
        // 在定义域内随机采样
        dom.sample(x, &mut self.rng);

        // 计算并返回函数值
        Ok(f.apply(x))
    }
}

// 定义曲线拟合的代价函数
struct CurveFittingCost {
    // 数据点的x值
    x: f64, 
    // 数据点的y值
    y: f64, 
}

impl CurveFittingCost {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

// 实现Problem trait，定义问题的域
impl Problem for CurveFittingCost {
    // 字段类型为f64
    type Field = f64; 

    // 返回问题的定义域
    fn domain(&self) -> Domain<Self::Field> {
        // 定义域为三维矩形
        Domain::rect(vec![-10.0, -10.0, -10.0], vec![10.0, 10.0, 10.0]) 
    }
}

// 实现Function trait，定义代价函数
impl Function for CurveFittingCost {
    fn apply<Sx>(&self, abc: &na::Vector<Self::Field, Dyn, Sx>) -> Self::Field
    where
        Sx: na::Storage<Self::Field, Dyn> + IsContiguous, // 要求存储类型是连续的
    {
        // 计算残差：y - (a*x^2 + b*x + c).exp()
        let residual = self.y - (abc[0] * self.x * self.x + abc[1] * self.x + abc[2]).exp();
        // 返回残差的平方
        residual * residual 
    }
}

fn main() {
    // 真实参数值
    let ar = 1.0;
    let br = 2.0;
    let cr = 1.0;

    // 初始估计参数值
    let ae = 2.0;
    let be = -1.0;
    let ce = 5.0;

    // 数据点数量
    let n = 100; 
    // 噪声标准差
    let w_sigma = 1.0; 
    // 噪声标准差的倒数
    let inv_sigma = 1.0 / w_sigma; 
    // 初始化随机数生成器
    let mut rng = Rng::new(); 

    // 生成数据点
    let mut x_data = Vec::new();
    let mut y_data = Vec::new();

    for i in 0..n {
        // x值从0到1
        let x = i as f64 / 100.0; 
        x_data.push(x);
        // y值为真实函数值加上噪声
        y_data.push((ar * x * x + br * x + cr).exp() + rng.f64() * w_sigma * w_sigma);
    }

    // 初始化参数向量
    let mut abc = na::Vector3::new(ae, be, ce);

    // 创建代价函数实例
    let f = CurveFittingCost::new(x_data[0], y_data[0]);
    // 创建优化器驱动
    let mut optimizer = OptimizerDriver::builder(&f)
        // 使用随机优化算法
        .with_algo(|_, _| Random::new(Rng::new())) 
        .build();

    // 运行优化器
    optimizer
        .find(|state| {
            // 打印当前函数值和参数值
            println!("f(x) = {}\tx = {:?}", state.fx(), state.x());
            // 迭代100次后停止
            state.iter() >= 100 
        })
        .unwrap();

    // 打印估计的参数值
    println!("estimated a, b, c = {:?}", abc);
}