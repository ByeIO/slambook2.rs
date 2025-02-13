#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]

//! 设置g2o图优化类型

// 使用类型别名
use super::preclude::*;

// 标准库
use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::cell::{
    RefMut, Ref, RefCell
};
use std::sync::{Weak, Arc, Mutex, RwLock};
use std::collections::HashMap;
use std::ops::{Deref, DerefMut};
use std::borrow::{Borrow, BorrowMut};

// 定义符号变量
assign_symbols!(X: SE3);
assign_symbols!(Y: VectorVar3);
assign_symbols!(Z: VectorVar2);

// 位姿顶点
pub struct VertexPose {
    pub id: usize,
    // 使用 RwLock 包装 SE3
    pub estimate: RwLock<SE3>, 
}

impl VertexPose {
    pub fn new(id: usize, estimate: SE3) -> Self {
        Self {
            id,
            estimate: RwLock::new(estimate),
        }
    }

    // 将顶点估计值设置为初始值
    pub fn set_to_origin(&mut self) {
        let mut estimate = self.estimate.write().unwrap();
        *estimate = SE3::identity();
    }

    // 对SE3进行左乘更新
    pub fn oplus(&mut self, update: &Vector6<f64>) {
        let mut estimate = self.estimate.write().unwrap();
        *estimate = SE3::exp(update.into()) * estimate.clone();
    }

    // 读取顶点数据（这里暂时简单返回成功）
    pub fn read(&mut self, _is: &mut dyn BufRead) -> io::Result<()> {
        Ok(())
    }

    // 写入顶点数据（这里暂时简单返回成功）
    pub fn write(&self, _os: &mut dyn Write) -> io::Result<()> {
        Ok(())
    }
}

// 路标顶点
pub struct VertexXYZ {
    pub id: usize,
    pub estimate: RwLock<Vec3>,  // 使用 RwLock 包装 Vec3
}

impl VertexXYZ {
    pub fn new(id: usize, estimate: Vec3) -> Self {
        Self {
            id,
            estimate: RwLock::new(estimate),
        }
    }

    // 将顶点估计值设置为零向量
    pub fn set_to_origin(&mut self) {
        let mut estimate = self.estimate.write().unwrap();
        *estimate = Vec3::zeros();
    }

    // 更新顶点估计值
    pub fn oplus(&mut self, update: &Vec3) {
        let mut estimate = self.estimate.write().unwrap();
        *estimate += update;
    }

    // 读取顶点数据（这里暂时简单返回成功）
    pub fn read(&mut self, _is: &mut dyn BufRead) -> io::Result<()> {
        Ok(())
    }

    // 写入顶点数据（这里暂时简单返回成功）
    pub fn write(&self, _os: &mut dyn Write) -> io::Result<()> {
        Ok(())
    }
}

// 仅估计位姿的一元边
pub struct EdgeProjectionPoseOnly {
    pub vertex: usize,
    pub measurement: Vec2,
    pub pos3d: Vec3,
    pub K: Mat33,
}

impl EdgeProjectionPoseOnly {
    pub fn new(vertex: usize, measurement: Vec2, pos3d: Vec3, K: Mat33) -> Self {
        Self {
            vertex,
            measurement,
            pos3d,
            K,
        }
    }

    // 计算误差
    pub fn compute_error(&self, v: &SE3) -> Vec2 {
        // 使用 `apply` 方法将 SE3 作用于向量
        let pos_cam = v.apply(self.pos3d.as_view());
        let pos_pixel = self.K * pos_cam;
        let pos_pixel = pos_pixel / pos_pixel[2];
        self.measurement - pos_pixel.fixed_rows::<2>(0)
    }

    // 计算雅可比矩阵
    pub fn linearize_oplus(&self, v: &SE3) -> Matrix2x6<f64> {
        let pos_cam = v.apply(self.pos3d.as_view());
        let fx = self.K[(0, 0)];
        let fy = self.K[(1, 1)];
        // 修改变量名，避免与符号变量冲突
        let x = pos_cam[0];
        let y = pos_cam[1];
        let z = pos_cam[2];
        let Zinv = 1.0 / (z + 1e-18);
        let Zinv2 = Zinv * Zinv;
        let mut jacobian = Matrix2x6::zeros();
        jacobian.row_mut(0).copy_from_slice(&[
            -fx * Zinv, 0., fx * x * Zinv2, fx * x * y * Zinv2,
            -fx - fx * x * x * Zinv2, fx * y * Zinv
        ]);
        jacobian.row_mut(1).copy_from_slice(&[
            0., -fy * Zinv, fy * y * Zinv2, fy + fy * y * y * Zinv2,
            -fy * x * y * Zinv2, -fy * x * Zinv
        ]);
        jacobian
    }

    // 读取边数据（这里暂时简单返回成功）
    pub fn read(&mut self, _is: &mut dyn BufRead) -> io::Result<()> {
        Ok(())
    }

    // 写入边数据（这里暂时简单返回成功）
    pub fn write(&self, _os: &mut dyn Write) -> io::Result<()> {
        Ok(())
    }
}

// 带有地图和位姿的二元边
pub struct EdgeProjection {
    pub vertex_pose: usize,
    pub vertex_xyz: usize,
    pub measurement: Vec2,
    pub K: Mat33,
    pub cam_ext: SE3,
}

impl EdgeProjection {
    // 构造时传入相机内外参
    pub fn new(vertex_pose: usize, vertex_xyz: usize, measurement: Vec2, K: Mat33, cam_ext: SE3) -> Self {
        Self {
            vertex_pose,
            vertex_xyz,
            measurement,
            K,
            cam_ext,
        }
    }

    // 计算误差
    pub fn compute_error(&self, v0: &SE3, v1: &Vec3) -> Vec2 {
        // 使用 `apply` 方法将 SE3 作用于向量
        let pos_cam = self.cam_ext.apply(v0.apply(v1.as_view()).as_view());
        let pos_pixel = self.K * pos_cam;
        let pos_pixel = pos_pixel / pos_pixel[2];
        self.measurement - pos_pixel.fixed_rows::<2>(0)
    }

    // 计算雅可比矩阵
    pub fn linearize_oplus(&self, v0: &SE3, v1: &Vec3) -> (Matrix2x6<f64>, Matrix2x3<f64>) {
        let pos_cam = self.cam_ext.apply(v0.apply(v1.as_view()).as_view());
        let fx = self.K[(0, 0)];
        let fy = self.K[(1, 1)];
        // 修改变量名，避免与符号变量冲突
        let x = pos_cam[0];
        let y = pos_cam[1];
        let z = pos_cam[2];
        let Zinv = 1.0 / (z + 1e-18);
        let Zinv2 = Zinv * Zinv;
        let mut jacobian_xi = Matrix2x6::zeros();
        jacobian_xi.row_mut(0).copy_from_slice(&[
            -fx * Zinv, 0., fx * x * Zinv2, fx * x * y * Zinv2,
            -fx - fx * x * x * Zinv2, fx * y * Zinv
        ]);
        jacobian_xi.row_mut(1).copy_from_slice(&[
            0., -fy * Zinv, fy * y * Zinv2, fy + fy * y * y * Zinv2,
            -fy * x * y * Zinv2, -fy * x * Zinv
        ]);
        // 使用 `to_matrix` 方法获取旋转矩阵
        let jacobian_xj = jacobian_xi.fixed_view::<2, 3>(0, 0) *
            self.cam_ext.to_matrix().fixed_view::<3, 3>(0, 0) * v0.to_matrix().fixed_view::<3, 3>(0, 0);
        (jacobian_xi, jacobian_xj.into())
    }

    // 读取边数据（这里暂时简单返回成功）
    pub fn read(&mut self, _is: &mut dyn BufRead) -> io::Result<()> {
        Ok(())
    }

    // 写入边数据（这里暂时简单返回成功）
    pub fn write(&self, _os: &mut dyn Write) -> io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests1 {
    // 引入待测试代码所在的模块
    use super::*;
    use rand_distr::num_traits;
    use std::fmt::Debug;
    use nalgebra::{OMatrix, DMatrix, Dyn, Matrix};
    use factrs::linalg::VectorViewX;

    // 辅助函数：检查 SE3 实例是否相等
    fn se3_equals(se3_1: &SE3, se3_2: &SE3) -> bool {
        se3_1.rot().to_matrix() == se3_2.rot().to_matrix()
            && se3_1.xyz().as_slice() == se3_2.xyz().as_slice()
    }

    // 辅助函数：检查矩阵的所有元素是否为有限值
    fn matrix_is_finite<T: Copy + num_traits::Float + Debug + 'static>(
        matrix: &OMatrix<T, Dyn, Dyn>,
    ) -> bool {
        matrix.iter().all(|&x| x.is_finite())
    }

    // 辅助函数：将 Vector6 转换为 VectorViewX
    fn vector6_to_viewx<T: Numeric>(v: &Vector6<T>) -> VectorViewX<T> {
        VectorViewX::from_slice(v.as_slice(), 6)
    }

    // 测试 VertexPose 结构体的 new 方法
    #[test]
    fn test_vertex_pose_new() {
        let id = 1;
        let estimate = SE3::identity();
        let vertex = VertexPose::new(id, estimate.clone());
        // 验证 id 是否正确
        assert_eq!(vertex.id, id);
        // 验证 estimate 是否正确
        let estimate_read = vertex.estimate.read().unwrap();
        assert!(se3_equals(&estimate_read, &estimate));
    }

    // 测试 VertexPose 结构体的 set_to_origin 方法
    #[test]
    fn test_vertex_pose_set_to_origin() {
        let id = 1;
        let translation = Vec3::new(1.0, 2.0, 3.0);
        let estimate = SE3::from_rot_trans(SO3::identity(), translation);
        let mut vertex = VertexPose::new(id, estimate);
        vertex.set_to_origin();
        // 验证 estimate 是否被设置为单位矩阵
        let estimate_read = vertex.estimate.read().unwrap();
        assert!(se3_equals(&estimate_read, &SE3::identity()));
    }

    // 测试 VertexPose 结构体的 oplus 方法
    #[test]
    fn test_vertex_pose_oplus() {
        let id = 1;
        let estimate = SE3::identity();
        let mut vertex = VertexPose::new(id, estimate.clone());
        let update = Vector6::zeros();
        let update_view = vector6_to_viewx(&update);
        vertex.oplus(&update);
        // 验证 estimate 是否正确更新
        let expected = SE3::exp(update_view) * estimate;
        let estimate_read = vertex.estimate.read().unwrap();
        assert!(se3_equals(&estimate_read, &expected));
    }

    // 测试 VertexPose 结构体的 read 方法
    #[test]
    fn test_vertex_pose_read() {
        let id = 1;
        let estimate = SE3::identity();
        let mut vertex = VertexPose::new(id, estimate);
        let mut cursor = Cursor::new(Vec::new());
        // 验证 read 方法是否返回 Ok
        assert!(vertex.read(&mut cursor).is_ok());
    }

    // 测试 VertexPose 结构体的 write 方法
    #[test]
    fn test_vertex_pose_write() {
        let id = 1;
        let estimate = SE3::identity();
        let vertex = VertexPose::new(id, estimate);
        let mut cursor = Cursor::new(Vec::new());
        // 验证 write 方法是否返回 Ok
        assert!(vertex.write(&mut cursor).is_ok());
    }

    // 测试 VertexXYZ 结构体的 new 方法
    #[test]
    fn test_vertex_xyz_new() {
        let id = 1;
        let estimate = Vec3::zeros();
        let vertex = VertexXYZ::new(id, estimate);
        // 验证 id 是否正确
        assert_eq!(vertex.id, id);
        // 验证 estimate 是否正确
        let estimate_read = vertex.estimate.read().unwrap();
        assert_eq!(*estimate_read, estimate);
    }

    // 测试 VertexXYZ 结构体的 set_to_origin 方法
    #[test]
    fn test_vertex_xyz_set_to_origin() {
        let id = 1;
        let estimate = Vec3::new(1.0, 2.0, 3.0);
        let mut vertex = VertexXYZ::new(id, estimate);
        vertex.set_to_origin();
        // 验证 estimate 是否被设置为零向量
        let estimate_read = vertex.estimate.read().unwrap();
        assert_eq!(*estimate_read, Vec3::zeros());
    }

    // 测试 VertexXYZ 结构体的 oplus 方法
    #[test]
    fn test_vertex_xyz_oplus() {
        let id = 1;
        let estimate = Vec3::zeros();
        let mut vertex = VertexXYZ::new(id, estimate);
        let update = Vec3::new(1.0, 1.0, 1.0);
        vertex.oplus(&update);
        // 验证 estimate 是否正确更新
        let estimate_read = vertex.estimate.read().unwrap();
        assert_eq!(*estimate_read, estimate + update);
    }

    // 测试 VertexXYZ 结构体的 read 方法
    #[test]
    fn test_vertex_xyz_read() {
        let id = 1;
        let estimate = Vec3::zeros();
        let mut vertex = VertexXYZ::new(id, estimate);
        let mut cursor = Cursor::new(Vec::new());
        // 验证 read 方法是否返回 Ok
        assert!(vertex.read(&mut cursor).is_ok());
    }

    // 测试 VertexXYZ 结构体的 write 方法
    #[test]
    fn test_vertex_xyz_write() {
        let id = 1;
        let estimate = Vec3::zeros();
        let vertex = VertexXYZ::new(id, estimate);
        let mut cursor = Cursor::new(Vec::new());
        // 验证 write 方法是否返回 Ok
        assert!(vertex.write(&mut cursor).is_ok());
    }

    // 测试 EdgeProjectionPoseOnly 结构体的 new 方法
    #[test]
    fn test_edge_projection_pose_only_new() {
        let vertex = 1;
        let measurement = Vec2::zeros();
        let pos3d = Vec3::zeros();
        let K = Mat33::zeros();
        let edge = EdgeProjectionPoseOnly::new(vertex, measurement, pos3d, K);
        // 验证 vertex 是否正确
        assert_eq!(edge.vertex, vertex);
        // 验证 measurement 是否正确
        assert_eq!(edge.measurement, measurement);
        // 验证 pos3d 是否正确
        assert_eq!(edge.pos3d, pos3d);
        // 验证 K 是否正确
        assert_eq!(edge.K, K);
    }

    // 测试 EdgeProjectionPoseOnly 结构体的 compute_error 方法
    #[test]
    fn test_edge_projection_pose_only_compute_error() {
        let vertex = 1;
        let measurement = Vec2::zeros();
        let pos3d = Vec3::zeros();
        let K = Mat33::zeros();
        let edge = EdgeProjectionPoseOnly::new(vertex, measurement, pos3d, K);
        let v = SE3::identity();
        let error = edge.compute_error(&v);
        // 将固定大小的矩阵手动复制到动态大小的矩阵
        let rows = error.nrows();
        let cols = error.ncols();
        let mut dynamic_error = DMatrix::zeros(rows, cols);
        for (i, mut row) in dynamic_error.row_iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = error[(i, j)];
            }
        }
        // 验证误差计算是否正确
        assert!(matrix_is_finite(&dynamic_error));
    }

    // 测试 EdgeProjectionPoseOnly 结构体的 linearize_oplus 方法
    #[test]
    fn test_edge_projection_pose_only_linearize_oplus() {
        let vertex = 1;
        let measurement = Vec2::zeros();
        let pos3d = Vec3::zeros();
        let K = Mat33::zeros();
        let edge = EdgeProjectionPoseOnly::new(vertex, measurement, pos3d, K);
        let v = SE3::identity();
        let jacobian = edge.linearize_oplus(&v);
        // 将固定大小的矩阵手动复制到动态大小的矩阵
        let rows = jacobian.nrows();
        let cols = jacobian.ncols();
        let mut dynamic_jacobian = DMatrix::zeros(rows, cols);
        for (i, mut row) in dynamic_jacobian.row_iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = jacobian[(i, j)];
            }
        }
        // 验证雅可比矩阵计算是否正确
        assert!(matrix_is_finite(&dynamic_jacobian));
    }

    // 测试 EdgeProjectionPoseOnly 结构体的 read 方法
    #[test]
    fn test_edge_projection_pose_only_read() {
        let vertex = 1;
        let measurement = Vec2::zeros();
        let pos3d = Vec3::zeros();
        let K = Mat33::zeros();
        let mut edge = EdgeProjectionPoseOnly::new(vertex, measurement, pos3d, K);
        let mut cursor = Cursor::new(Vec::new());
        // 验证 read 方法是否返回 Ok
        assert!(edge.read(&mut cursor).is_ok());
    }

    // 测试 EdgeProjectionPoseOnly 结构体的 write 方法
    #[test]
    fn test_edge_projection_pose_only_write() {
        let vertex = 1;
        let measurement = Vec2::zeros();
        let pos3d = Vec3::zeros();
        let K = Mat33::zeros();
        let edge = EdgeProjectionPoseOnly::new(vertex, measurement, pos3d, K);
        let mut cursor = Cursor::new(Vec::new());
        // 验证 write 方法是否返回 Ok
        assert!(edge.write(&mut cursor).is_ok());
    }

    // 测试 EdgeProjection 结构体的 new 方法
    #[test]
    fn test_edge_projection_new() {
        let vertex_pose = 1;
        let vertex_xyz = 2;
        let measurement = Vec2::zeros();
        let K = Mat33::zeros();
        let cam_ext = SE3::identity();
        let edge = EdgeProjection::new(vertex_pose, vertex_xyz, measurement, K, cam_ext.clone());
        // 验证 vertex_pose 是否正确
        assert_eq!(edge.vertex_pose, vertex_pose);
        // 验证 vertex_xyz 是否正确
        assert_eq!(edge.vertex_xyz, vertex_xyz);
        // 验证 measurement 是否正确
        assert_eq!(edge.measurement, measurement);
        // 验证 K 是否正确
        assert_eq!(edge.K, K);
        // 验证 cam_ext 是否正确
        assert!(se3_equals(&edge.cam_ext, &cam_ext));
    }

    // 测试 EdgeProjection 结构体的 compute_error 方法
    #[test]
    fn test_edge_projection_compute_error() {
        let vertex_pose = 1;
        let vertex_xyz = 2;
        let measurement = Vec2::zeros();
        let K = Mat33::zeros();
        let cam_ext = SE3::identity();
        let edge = EdgeProjection::new(vertex_pose, vertex_xyz, measurement, K, cam_ext);
        let v0 = SE3::identity();
        let v1 = Vec3::zeros();
        let error = edge.compute_error(&v0, &v1);
        // 将固定大小的矩阵手动复制到动态大小的矩阵
        let rows = error.nrows();
        let cols = error.ncols();
        let mut dynamic_error = DMatrix::zeros(rows, cols);
        for (i, mut row) in dynamic_error.row_iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = error[(i, j)];
            }
        }
        // 验证误差计算是否正确
        assert!(matrix_is_finite(&dynamic_error));
    }

    // 测试 EdgeProjection 结构体的 linearize_oplus 方法
    #[test]
    fn test_edge_projection_linearize_oplus() {
        let vertex_pose = 1;
        let vertex_xyz = 2;
        let measurement = Vec2::zeros();
        let K = Mat33::zeros();
        let cam_ext = SE3::identity();
        let edge = EdgeProjection::new(vertex_pose, vertex_xyz, measurement, K, cam_ext);
        let v0 = SE3::identity();
        let v1 = Vec3::zeros();
        let (jacobian_xi, jacobian_xj) = edge.linearize_oplus(&v0, &v1);
        // 将固定大小的矩阵手动复制到动态大小的矩阵
        let rows_xi = jacobian_xi.nrows();
        let cols_xi = jacobian_xi.ncols();
        let mut dynamic_jacobian_xi = DMatrix::zeros(rows_xi, cols_xi);
        for (i, mut row) in dynamic_jacobian_xi.row_iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = jacobian_xi[(i, j)];
            }
        }

        let rows_xj = jacobian_xj.nrows();
        let cols_xj = jacobian_xj.ncols();
        let mut dynamic_jacobian_xj = DMatrix::zeros(rows_xj, cols_xj);
        for (i, mut row) in dynamic_jacobian_xj.row_iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = jacobian_xj[(i, j)];
            }
        }

        // 验证雅可比矩阵计算是否正确
        assert!(matrix_is_finite(&dynamic_jacobian_xi));
        assert!(matrix_is_finite(&dynamic_jacobian_xj));
    }

    // 测试 EdgeProjection 结构体的 read 方法
    #[test]
    fn test_edge_projection_read() {
        let vertex_pose = 1;
        let vertex_xyz = 2;
        let measurement = Vec2::zeros();
        let K = Mat33::zeros();
        let cam_ext = SE3::identity();
        let mut edge = EdgeProjection::new(vertex_pose, vertex_xyz, measurement, K, cam_ext);
        let mut cursor = Cursor::new(Vec::new());
        // 验证 read 方法是否返回 Ok
        assert!(edge.read(&mut cursor).is_ok());
    }

    // 测试 EdgeProjection 结构体的 write 方法
    #[test]
    fn test_edge_projection_write() {
        let vertex_pose = 1;
        let vertex_xyz = 2;
        let measurement = Vec2::zeros();
        let K = Mat33::zeros();
        let cam_ext = SE3::identity();
        let edge = EdgeProjection::new(vertex_pose, vertex_xyz, measurement, K, cam_ext);
        let mut cursor = Cursor::new(Vec::new());
        // 验证 write 方法是否返回 Ok
        assert!(edge.write(&mut cursor).is_ok());
    }
}
