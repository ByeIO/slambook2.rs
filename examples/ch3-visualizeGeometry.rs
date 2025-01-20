#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(unused_assignments)]
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] 
#![allow(rustdoc::missing_crate_level_docs)]
#![allow(unsafe_code)]
#![allow(clippy::undocumented_unsafe_blocks)]

use nalgebra::{
    Matrix3, Vector3, UnitQuaternion, 
    Quaternion, Isometry3, Rotation3, UnitComplex, Rotation2, Unit,
    Translation3, Perspective3, Orthographic3, Vector4, Point3, Const,
    ArrayStorage, Matrix4, ViewStorage
};

// 绘图和界面库
use three_d::*; 
use three_d::egui::*;

use env_logger::init;

use std::fmt;
use std::sync::Arc;
use std::any::type_name;

// 1. 定义一个结构体，用于表示旋转矩阵
#[derive(Debug)]
#[derive(PartialEq)]
#[derive(Default)]
struct RotationMatrix {
    matrix: Matrix3<f64>,
}

// 实现Display trait，以便输出旋转矩阵
impl fmt::Display for RotationMatrix {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self.matrix)
    }
}

// 2. 定义一个结构体，用于表示平移向量
#[derive(Debug)]
#[derive(PartialEq)]
#[derive(Default)]
struct TranslationVector {
    trans: Vector3<f64>,
}
// 实现Display trait，以便输出平移向量
impl fmt::Display for TranslationVector {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "=[{}, {}, {}]", self.trans.x, self.trans.y, self.trans.z)
    }
}
// 3. 定义一个结构体，用于表示四元数
#[derive(Debug)]
#[derive(PartialEq)]
#[derive(Default)]
struct QuaternionDraw {
    q: UnitQuaternion<f64>,
}
// 实现Display trait，以便输出四元数
impl fmt::Display for QuaternionDraw {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let i = self.q.quaternion().i;
        let j = self.q.quaternion().j;
        let k = self.q.quaternion().k;
        let w = self.q.quaternion().w;
        write!(f, "=[{}, {}, {}, {}]", i, j, k, w)
    }
}

 /* start 测试 */ 
fn test() {
    // 创建一个单位旋转矩阵
    let rotation_matrix = RotationMatrix {
        matrix: Matrix3::identity(),
    };
    // 测试矩阵是否为单位矩阵
    assert_eq!(rotation_matrix.matrix, Matrix3::identity(), "The matrix should be the identity matrix.");
    // 测试矩阵的行列式是否为1
    assert_eq!(rotation_matrix.matrix.determinant(), 1.0, "The determinant of the identity matrix should be 1.");
    // 打印旋转矩阵
    println!("{}", rotation_matrix);

    // 创建一个平移向量
    let translation_vector = TranslationVector {
        trans: Vector3::new(1.0, 2.0, 3.0),
    };
    // 定义一个预期的平移向量
    let expected_translation_vector = TranslationVector {
        trans: Vector3::new(1.0, 2.0, 3.0),
    };
    // 测试平移向量是否相等
    assert_eq!(translation_vector, expected_translation_vector, "The translation vectors should be equal.");
    // 打印平移向量
    println!("{}", translation_vector);

    // 创建一个单位四元数
    let quaternion_draw = QuaternionDraw {
        q: UnitQuaternion::identity(),
    };
    // 定义一个预期的单位四元数
    let expected_quaternion_draw = QuaternionDraw {
        q: UnitQuaternion::identity(),
    };
    // 测试四元数是否相等
    assert_eq!(quaternion_draw, expected_quaternion_draw, "The quaternions should be equal.");
    // 打印四元数
    println!("{}", quaternion_draw);
}
/* end 测试 */

/* start 绘制界面和绘图 */
pub fn main() {
    let window = three_d::Window::new(WindowSettings {
        title: "Shapes!".to_string(),
        max_size: Some((1280, 720)),
        ..Default::default()
    })
    .unwrap();
    let context = window.gl();

    let mut camera = three_d::Camera::new_perspective(
        window.viewport(),
        vec3(5.0, 2.0, 2.5),
        vec3(0.0, 0.0, -0.5),
        vec3(0.0, 1.0, 0.0),
        degrees(45.0),
        0.1,
        1000.0,
    );
    let mut control = three_d::OrbitControl::new(camera.target(), 1.0, 100.0);

    /* start 绘制立体图形 */
    let mut sphere = three_d::Gm::new(
        three_d::Mesh::new(&context, &CpuMesh::sphere(16)),
        three_d::PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 255,
                    g: 0,
                    b: 0,
                    a: 200,
                },
                ..Default::default()
            },
        ),
    );
    sphere.set_transformation(Mat4::from_translation(vec3(0.0, 1.3, 0.0)) * Mat4::from_scale(0.2));
    let mut cylinder = three_d::Gm::new(
        three_d::Mesh::new(&context, &CpuMesh::cylinder(16)),
        three_d::PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 0,
                    g: 255,
                    b: 0,
                    a: 200,
                },
                ..Default::default()
            },
        ),
    );
    cylinder
        .set_transformation(Mat4::from_translation(vec3(1.3, 0.0, 0.0)) * Mat4::from_scale(0.2));
    let mut cube = three_d::Gm::new(
        three_d::Mesh::new(&context, &CpuMesh::cube()),
        three_d::PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 0,
                    g: 0,
                    b: 255,
                    a: 100,
                },
                ..Default::default()
            },
        ),
    );
    cube.set_transformation(Mat4::from_translation(vec3(0.0, 0.0, 1.3)) * Mat4::from_scale(0.2));
    let axes = three_d::Axes::new(&context, 0.1, 2.0);
    let bounding_box_sphere = Gm::new(
        BoundingBox::new(&context, sphere.aabb()),
        ColorMaterial {
            color: Srgba::BLACK,
            ..Default::default()
        },
    );
    let bounding_box_cube = three_d::Gm::new(
        three_d::BoundingBox::new(&context, cube.aabb()),
        ColorMaterial {
            color: Srgba::BLACK,
            ..Default::default()
        },
    );
    let bounding_box_cylinder = three_d::Gm::new(
        three_d::BoundingBox::new(&context, cylinder.aabb()),
        ColorMaterial {
            color: Srgba::BLACK,
            ..Default::default()
        },
    );

    let light0 = DirectionalLight::new(&context, 1.0, Srgba::WHITE, vec3(0.0, -0.5, -0.5));
    let light1 = DirectionalLight::new(&context, 1.0, Srgba::WHITE, vec3(0.0, 0.5, 0.5));
    /* end 绘制立体图形 */

    let mut gui = three_d::GUI::new(&context);
    // main loop
    window.render_loop(move |mut frame_input| {
        let mut panel_width = 0.0;
        gui.update(
            &mut frame_input.events,
            frame_input.accumulated_time,
            frame_input.viewport,
            frame_input.device_pixel_ratio,
            |gui_context| {

                SidePanel::left("side_panel").show(gui_context, |ui| {
                    ui.heading("Camera Pose");
                    // 显示相机的平移向量
                    let translation_vector = camera.position();
                    ui.label(format!("Translation Vector: {:?}", translation_vector));
                    // 获取相机的视图矩阵
                    let view_matrix = camera.view();

                    /* start 打印camera.view()返回值的数据类型 */
                    // fn type_name_of<T>(_: &T) -> &'static str {
                    //     std::any::type_name::<T>()
                    // }
                    // (|view_matrix| println!("typeof view_matrix is {:?}", type_name_of(view_matrix)))(&view_matrix);
                    /* end 打印camera.view()返回值的数据类型 */

                    // 从视图矩阵中提取旋转矩阵
                    // view_matrix:cgmath::Matrix4<f32>转为nal_view_matrix:nalgebra::Matrix4<f32>数据
                    let nal_view_matrix = nalgebra::Matrix4::new(
                        view_matrix.x.x, view_matrix.x.y, view_matrix.x.z, view_matrix.x.w,
                        view_matrix.y.x, view_matrix.y.y, view_matrix.y.z, view_matrix.y.w,
                        view_matrix.z.x, view_matrix.z.y, view_matrix.z.z, view_matrix.z.w,
                        view_matrix.w.x, view_matrix.w.y, view_matrix.w.z, view_matrix.w.w,
                    );
                    let rotation_matrix = nal_view_matrix.try_inverse().unwrap_or_else(|| Matrix4::identity());
                    // let rotation_matrix = nalgebra::Matrix4::<f64>::identity();
                    
                    // 从 rotation_matrix 中提取 3x3 的视图
                    let rotation_matrix_view = rotation_matrix.fixed_view::<3, 3>(0, 0);
                    // 创建一个新的 nalgebra::Matrix3<f32> 实例
                    let rotation_matrix3 = nalgebra::Matrix3::from(rotation_matrix_view);
                    ui.label(format!("Rotation Matrix: {:?}", rotation_matrix3));

                    // 从旋转矩阵中提取四元数
                    // let quaternion = Quaternion::new(1.0, 0.0, 0.0, 0.0); // 提供了合适的数值
                    // 注意：这里假设 rotation_matrix3 是一个纯旋转矩阵，没有平移部分
                    let unit_quaternion = UnitQuaternion::from_matrix(&rotation_matrix3);
                    ui.label(format!("Quaternion: {:?}", unit_quaternion));

                    // 从四元数中提取欧拉角
                    let euler_angles = unit_quaternion.euler_angles();
                    ui.label(format!("Euler Angles: {:?}", euler_angles));

                }); // end SidePanel show

                panel_width = gui_context.used_rect().width();
            }, // end gui_context

        ); // end gui.update

        // 更新相机视图
        camera.set_viewport(Viewport {
            x: (panel_width * frame_input.device_pixel_ratio) as i32,
            y: 0,
            width: frame_input.viewport.width
                - (panel_width * frame_input.device_pixel_ratio) as u32,
            height: frame_input.viewport.height,
        });
        control.handle_events(&mut camera, &mut frame_input.events);
        // 绘制场景
        frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.8, 0.8, 0.8, 1.0, 1.0))
            .render(
                &camera,
                sphere
                    .into_iter()
                    .chain(&cylinder)
                    .chain(&cube)
                    .chain(&axes)
                    .chain(&bounding_box_sphere)
                    .chain(&bounding_box_cube)
                    .chain(&bounding_box_cylinder),
                &[&light0, &light1],
            )
            .write(|| gui.render())
            .unwrap();
        FrameOutput::default()


    });
}
/* end 绘制界面和绘图 */