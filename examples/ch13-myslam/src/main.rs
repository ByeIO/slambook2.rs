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

//! 视觉里程计程序, 包含管理局部的机器人轨迹与路标点,
//! 以及对图像进行连续追踪.

// 引入命令行参数解析库
use clap::Parser;

// 内部库
mod myslam;
use myslam::*;
use myslam::visual_odometry::VisualOdometry;

// 定义命令行参数
#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// 配置文件路径
    #[clap(short, long, default_value = "./assets/default.yaml")]
    pub config_file: String,
}

fn main() {
    // 解析命令行参数
    let args = Args::parse();
    
    println!("输入的配置文件路径为: {}\n", args.config_file);

    // 创建 VisualOdometry 实例，传入配置文件路径
    let mut vo = VisualOdometry::new(args.config_file);

    // 调用初始化函数，并断言初始化成功
    // assert!(vo.init() == true);

    // 启动视觉里程计
    // vo.run();

    println!("功能还没有实现完全, 勉强中...(ง •̀_•́)ง..., \n程序即将退出!\n");
    
    // 程序正常退出
    std::process::exit(0);
}
