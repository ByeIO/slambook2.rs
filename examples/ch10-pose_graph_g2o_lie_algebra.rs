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

use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;

use std::cell::{
    RefMut, Ref, RefCell
};

use nalgebra::{
    Quaternion, Vector3, Matrix3, UnitQuaternion, 
    Isometry3, Translation3, Const,
    Matrix, Point3, ViewStorage, Rotation3,
    Matrix3x1, VectorView3, DMatrix, DVector, SVector,
    Vector6, Matrix6
};

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

/**
 * 本程序演示如何用factrs solver进行位姿图优化
 * sphere.g2o是人工生成的一个Pose graph，我们来优化它。
 * 尽管可以直接通过load函数读取整个图，但我们还是自己来实现读取代码，以期获得更深刻的理解
 * 本程序使用李代数(而不仅仅是四元数)表达位姿图，节点和边的方式为自定义
**/

// 定义符号变量
assign_symbols!(X: SE3);

// 给定误差求J_R^{-1}的近似
fn jr_inv(e: &SE3) -> Matrix6<f64> {
    let mut j = Matrix6::identity();
    j.fixed_view_mut::<3, 3>(0, 0).copy_from(&SO3::hat(e.rot().log().as_view()));
    j.fixed_view_mut::<3, 3>(0, 3).copy_from(&SO3::hat(e.xyz()));
    j.fixed_view_mut::<3, 3>(3, 0).copy_from(&Matrix3::zeros());
    j.fixed_view_mut::<3, 3>(3, 3).copy_from(&SO3::hat(e.rot().log().as_view()));
    // 返回值
    j
}

// 李代数顶点
struct VertexSE3LieAlgebra {
    id: usize,
    estimate: RefCell<SE3>,  // 使用 RefCell 包装 SE3
}

impl VertexSE3LieAlgebra {
    fn new(id: usize, estimate: SE3) -> Self {
        Self {
            id,
            estimate: RefCell::new(estimate),
        }
    }

    fn set_to_origin(&mut self) {
        *self.estimate.borrow_mut() = SE3::identity();
    }

    fn oplus(&mut self, update: &Vector6<f64>) {
        let mut estimate = self.estimate.borrow_mut();
        *estimate = SE3::exp(update.into()) * estimate.clone();
    }

    fn read(&mut self, is: &mut dyn BufRead) -> io::Result<()> {
        let mut line = String::new();
        is.read_line(&mut line)?;
        let data: Vec<f64> = line
            .split_whitespace()
            .map(|s| s.parse().unwrap())
            .collect();

        let r = SO3::from_xyzw(data[3], data[4], data[5], data[6]);
        let t = Vector3::new(data[0], data[1], data[2]);
        *self.estimate.borrow_mut() = SE3::from_rot_trans(r, t);
        Ok(())
    }

    fn write(&self, os: &mut dyn Write) -> io::Result<()> {
        let estimate = self.estimate.borrow();
        let t = estimate.to_matrix();
        let q = estimate.clone();

        // 正确访问矩阵元素
        writeln!(
            os,
            "{} {} {} {} {} {} {} {}",
            self.id,
            // 访问矩阵元素
            t[(0, 0)], t[(0, 1)], t[(0, 2)],
            // 访问四元数元素
            q.rot().x(), q.rot().y(), q.rot().z(), q.rot().w()
        )
    }
}

// 李代数的边
struct EdgeSE3LieAlgebra {
    vertex1: usize,
    vertex2: usize,
    measurement: SE3,
    information: Matrix6<f64>,
}

impl EdgeSE3LieAlgebra {
    fn new(vertex1: usize, vertex2: usize, measurement: SE3, information: Matrix6<f64>) -> Self {
        Self {
            vertex1,
            vertex2,
            measurement,
            information,
        }
    }

    fn compute_error(&self, v1: &SE3, v2: &SE3) -> Vector6<f64> {
        let result = (self.measurement.inverse() * v1.inverse() * v2.clone()).log();
        Vector6::from_column_slice(result.as_slice())
    }

    // 雅可比计算
    fn linearize_oplus(&self, v1: &SE3, v2: &SE3) -> (Matrix6<f64>, Matrix6<f64>) {
        let j = jr_inv( &SE3::exp( (self.compute_error(v1, v2)).as_view() ) );
        let jacobian_xi = -j * v2.inverse().adjoint();
        let jacobian_xj = j * v2.inverse().adjoint();
        (jacobian_xi, jacobian_xj)
    }

    fn read(&mut self, is: &mut dyn BufRead) -> io::Result<()> {
        let mut line = String::new();
        is.read_line(&mut line)?;
        let data: Vec<f64> = line
            .split_whitespace()
            .map(|s| s.parse().unwrap())
            .collect();

        let r = SO3::from_xyzw(data[3], data[4], data[5], data[6]);
        let t = Vector3::new(data[0], data[1], data[2]);
        self.measurement = SE3::from_rot_trans(r, t);

        let mut info = Matrix6::zeros();
        for i in 0..6 {
            for j in i..6 {
                info[(i, j)] = data[7 + i * 6 + j];
                if i != j {
                    info[(j, i)] = info[(i, j)];
                }
            }
        }
        self.information = info;
        Ok(())
    }

    fn write(&self, os: &mut dyn Write) -> io::Result<()> {
        // 获取平移向量
        let t = self.measurement.xyz();
        // 获取四元数
        let q = self.measurement.rot();
        write!(
            os,
            "{} {} {} {} {} {} {} {} {}",
            self.vertex1, self.vertex2,
            t.x, t.y, t.z,
            q.x(), q.y(), q.z(), q.w()
        )?;

        for i in 0..6 {
            for j in i..6 {
                write!(os, " {}", self.information[(i, j)])?;
            }
        }
        writeln!(os)
    }
}

fn main() -> io::Result<()> {
    let file_path = "./assets/ch10-sphere.g2o";
    let file = File::open(file_path)?;
    let reader = io::BufReader::new(file);

    let mut graph = Graph::new();
    let mut values = Values::new();

    let mut vertex_cnt = 0;
    let mut edge_cnt = 0;

    let mut vertices: Vec<VertexSE3LieAlgebra> = Vec::new();
    let mut edges: Vec<EdgeSE3LieAlgebra> = Vec::new();

    for line in reader.lines() {
        let line = line?;
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.is_empty() {
            continue;
        }

        match parts[0] {
            // SE3 顶点
            "VERTEX_SE3:QUAT" => {
                let mut vertex = VertexSE3LieAlgebra::new(0, SE3::identity());
                vertex.read(&mut io::Cursor::new(line.as_bytes()))?;
                values.insert(X(vertex.id.try_into().unwrap()), vertex.estimate.borrow().clone());
                vertices.push(vertex);
                vertex_cnt += 1;
            }
            // SE3-SE3 边
            "EDGE_SE3:QUAT" => {
                let mut edge = EdgeSE3LieAlgebra::new(0, 0, SE3::identity(), Matrix6::zeros());
                edge.read(&mut io::Cursor::new(line.as_bytes()))?;
                edges.push(edge);
                edge_cnt += 1;
            }
            _ => {}
        } // end match

    } // end for

    println!("read total {} vertices, {} edges.", vertex_cnt, edge_cnt);

    println!("optimizing ...");

    let mut opt: LevenMarquardt = LevenMarquardt::new(graph.clone());
    let result = opt.optimize(values).expect("优化失败");

    println!("saving optimization results ...");

    save_g2o_file("ch10-pose_graph_g2o_lie_algebra-result.g2o", &result, &graph).expect("保存g2o文件失败");

    Ok(())
}

fn save_g2o_file(file_path: &str, values: &Values, graph: &Graph) -> io::Result<()> {
    let mut file = File::create(file_path)?;

    // // 保存顶点
    // for (key, value) in values.iter() {
    //     if let Some(vertex) = graph.vertex(key) {
    //         if let Some(se3) = value.downcast_ref::<SE3>() {
    //             let vertex_lie = VertexSE3LieAlgebra::new(vertex.id(), *se3);
    //             vertex_lie.write(&mut file)?;
    //         }
    //     }
    // }

    // // 保存边
    // for edge in graph.edges() {
    //     if let Some(edge_lie) = edge.downcast_ref::<EdgeSE3LieAlgebra>() {
    //         edge_lie.write(&mut file)?;
    //     }
    // }

    // TODO: 保存顶点和保存边
    

    // FIXME: 这里直接保存整个graph没有格式化了
    writeln!(file, "{:#?}", graph)?;

    Ok(())
}