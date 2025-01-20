#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(unused_assignments)]

use nalgebra::{
    Matrix3, Vector3, UnitQuaternion, 
    Quaternion, Isometry3, Rotation3, UnitComplex, Rotation2, Unit,
    Translation3, Perspective3, Orthographic3, Vector4, Point3, Const,
    ArrayStorage
};

fn main() {
    // 1. 3维旋转矩阵
    // 使用 Rotation3 创建 3D 旋转矩阵
    let mut rotation_matrix: Rotation3<f64> = Rotation3::identity();
    // 沿 Z 轴旋转 45 度
    let rotation_angle = std::f64::consts::PI / 4.0;
    let rotation_axis = Vector3::z();
    // 确保旋转轴是单位向量
    let unit_rotation_axis = Unit::new_unchecked(rotation_axis.normalize());
    // 使用 from_axis_angle 创建旋转
    let rotation = Rotation3::from_axis_angle(&unit_rotation_axis, rotation_angle);
    println!("rotation matrix =\n{:?}", rotation);

    // 也可以直接赋值
    rotation_matrix = rotation;
    // 使用旋转进行坐标变换
    let v = Vector3::new(1.0, 0.0, 0.0);
    let v_rotated = rotation * v;
    println!("(1,0,0) after rotation (by angle axis) = {:?}", v_rotated);
    // 或者用旋转矩阵
    let v_rotated_by_matrix = rotation_matrix * v;
    println!("(1,0,0) after rotation (by matrix) = {:?}", v_rotated_by_matrix);

    // 2. 欧拉角
    // 可以将旋转矩阵直接转换成欧拉角,默认为ZYX顺序，即yaw-pitch-roll顺序
    let euler_angles = rotation_matrix.euler_angles(); 
    println!("yaw pitch roll = {:?}", euler_angles);

    // 欧氏变换矩阵使用 Isometry3
    // 虽然称为3，实质上是4＊4的矩阵
    let mut t = Isometry3::identity();
    // 将Rotation3转换为UnitQuaternion
    let unit_quaternion = UnitQuaternion::from_rotation_matrix(&rotation);
    t.append_rotation_mut(&unit_quaternion); // 按照unit_quaternion进行旋转
    // 将Vector3转换为Translation3
    let translation = Translation3::from(Vector3::new(1.0, 3.0, 4.0));
    t.append_translation_mut(&translation); // 把平移向量设成(1,3,4)
    println!("Transform matrix = \n{:?}", t);

    // 用变换矩阵进行坐标变换
    let v_transformed = t * &v; // 相当于R*v+t
    println!("v tranformed = {:?}", v_transformed);

    // 3. 仿射变换
    // 创建一个仿射变换矩阵，包括旋转、平移和缩放
    let affine_transform = t.to_homogeneous(); // 获取4x4齐次变换矩阵
    println!("Affine transformation matrix = \n{:?}", affine_transform);
    // 将3D向量转换为4D齐次坐标
    let v_homogeneous = Vector4::new(v.x, v.y, v.z, 1.0);
    // 使用仿射变换进行坐标变换
    let v_affine_transformed = affine_transform * v_homogeneous;
    println!("v after affine transformation = {:?}", v_affine_transformed);

    // 4. 射影变换
    // 创建一个透视投影矩阵,焦距为1，宽高比为1，近剪裁平面为0.1，远剪裁平面为100
    let perspective_matrix = Perspective3::new(1.0, 1.0, 0.1, 100.0); 
    println!("Perspective projection matrix = \n{:?}", perspective_matrix);
    // 创建一个正交投影矩阵,左右边界为0和1，上下边界为0和1，近远剪裁平面为0.1和100
    let orthographic_matrix = Orthographic3::new(0.0, 1.0, 0.0, 1.0, 0.1, 100.0);
    println!("Orthographic projection matrix = \n{:?}", orthographic_matrix);
    // 使用透视投影进行坐标变换
    let v_point = Point3::new(1.0, 2.0, 3.0);
    let v_perspective_transformed = perspective_matrix.project_point(&v_point);
    println!("v after perspective transformation = {:?}", v_perspective_transformed);
    // 使用正交投影进行坐标变换
    let v_orthographic_transformed = orthographic_matrix.project_point(&v_point);
    println!("v_point after orthographic transformation = {:?}", v_orthographic_transformed);

    // 3. 四元数
    // 从旋转矩阵创建单位四元数
    let unit_quaternion = UnitQuaternion::from_rotation_matrix(&rotation);
    // 将单位四元数转换为普通四元数
    let q = unit_quaternion.quaternion();
    println!("quaternion from rotation matrix = {:?}", q);

    // 可以直接把旋转向量赋值给四元数，反之亦然
    let axis = Vector3::y_axis();
    let angle = std::f32::consts::FRAC_PI_2;
    let q2 = UnitQuaternion::from_axis_angle(&axis, angle);
    println!("quaternion from rotation vector = {:?}", q2);

    // 使用四元数旋转一个向量，使用重载的乘法即可
    let v2 = Vector3::new(1.0, 0.0, 0.0);
    let v2_rotated_by_quaternion = q2 * &v2; // 注意数学上是qvq^{-1}
    println!("(1,0,0) after rotation = {:?}", v2_rotated_by_quaternion);

    // 用常规向量乘法表示，则应该如下计算
    let qvq_inv = (*q2) * Quaternion::new(0.0, v2.x, v2.y, v2.z) * (*q2.inverse());
    println!("should be equal to {:?}", qvq_inv);
}