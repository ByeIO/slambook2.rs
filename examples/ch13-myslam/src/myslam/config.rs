#![allow(static_mut_refs)]
#![allow(dead_code)]

//! 获取配置参数

use yaml_rust2::{YamlLoader, Yaml};
use std::fs::File;
use std::io::Read;

// 配置类，单例模式
struct Config {
    // 存储解析后的 YAML 数据
    yaml: Yaml,
}

// 为 Config 实现 PartialEq trait，以允许在模式匹配中使用
impl PartialEq for Config {
    fn eq(&self, other: &Self) -> bool {
        self.yaml == other.yaml
    }
}

// 全局静态变量，用于存储单例实例
static mut INSTANCE: Option<Config> = None;

impl Config {
    // 设置配置文件
    pub fn set_parameter_file(filename: &str) -> bool {
        let mut file = match File::open(filename) {
            Ok(file) => file,
            Err(_) => {
                eprintln!("parameter file {} does not exist.", filename);
                return false;
            }
        };
        let mut contents = String::new();
        if let Err(_) = file.read_to_string(&mut contents) {
            eprintln!("Failed to read parameter file {}.", filename);
            return false;
        }
        let docs = match YamlLoader::load_from_str(&contents) {
            Ok(docs) => docs,
            Err(_) => {
                eprintln!("Failed to parse parameter file {}.", filename);
                return false;
            }
        };
        let doc = docs.into_iter().next().unwrap_or(Yaml::BadValue);
        // 使用 unsafe 块来修改静态变量
        unsafe {
            INSTANCE = Some(Config { yaml: doc });
        }
        true
    }

    // 获取配置项的值
    pub fn get(key: &str) -> Option<Yaml> {
        // 使用 unsafe 块来访问静态变量
        unsafe {
            match INSTANCE.as_ref() {
                Some(config) => {
                    let yaml_value = &config.yaml[key];
                    if yaml_value.is_badvalue() {
                        None
                    } else {
                        Some(yaml_value.clone())
                    }
                }
                None => None,
            }
        }
    }

    // 从 Yaml 转换为 String
    pub fn yaml_to_string(yaml: Yaml) -> Option<String> {
        yaml.as_str().map(|s| s.to_string())
    }

    // 从 Yaml 转换为 f64
    pub fn yaml_to_f64(yaml: Yaml) -> Option<f64> {
        yaml.as_f64()
    }

    // 从 Yaml 转换为 i64
    pub fn yaml_to_i64(yaml: Yaml) -> Option<i64> {
        yaml.as_i64()
    }
}

// 配置数据
struct ConfigData {
    filename: String,
    dataset_dir: Option<String>,
    fx: Option<f64>,
    fy: Option<f64>,
    cx: Option<f64>,
    cy: Option<f64>,
    num_features: Option<i64>,
    num_features_init: Option<i64>,
    num_features_tracking: Option<i64>,
}

// 静态配置数据实例
static mut CONFIG_DATA: Option<ConfigData> = None;

impl ConfigData {
    // 初始化配置数据
    pub fn init(filename: &str) {
        if Config::set_parameter_file(filename) {
            let dataset_dir = Config::get("dataset_dir").and_then(Config::yaml_to_string);
            let fx = Config::get("camera.fx").and_then(Config::yaml_to_f64);
            let fy = Config::get("camera.fy").and_then(Config::yaml_to_f64);
            let cx = Config::get("camera.cx").and_then(Config::yaml_to_f64);
            let cy = Config::get("camera.cy").and_then(Config::yaml_to_f64);
            let num_features = Config::get("num_features").and_then(Config::yaml_to_i64);
            let num_features_init = Config::get("num_features_init").and_then(Config::yaml_to_i64);
            let num_features_tracking = Config::get("num_features_tracking").and_then(Config::yaml_to_i64);

            let config_data = ConfigData {
                filename: filename.to_string(),
                dataset_dir,
                fx,
                fy,
                cx,
                cy,
                num_features,
                num_features_init,
                num_features_tracking,
            };

            unsafe {
                CONFIG_DATA = Some(config_data);
            }
        }
    }

    // 获取静态配置数据实例
    pub fn get_instance() -> Option<&'static ConfigData> {
        unsafe { CONFIG_DATA.as_ref() }
    }
}

// 单元测试模块
#[cfg(test)]
mod test1 {
    use super::*;

    #[test]
    fn test_config() {
        let filename = "./assets/default.yaml";
        ConfigData::init(filename);

        if let Some(config) = ConfigData::get_instance() {
            assert!(config.dataset_dir.is_some());
            assert!(config.fx.is_some());
            assert!(config.fy.is_some());
            assert!(config.cx.is_some());
            assert!(config.cy.is_some());
            assert!(config.num_features.is_some());
            assert!(config.num_features_init.is_some());
            assert!(config.num_features_tracking.is_some());
        }
    }
}
