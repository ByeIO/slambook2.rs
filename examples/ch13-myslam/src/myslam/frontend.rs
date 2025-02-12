#![allow(unused_imports)]
#![allow(dead_code)]
#![allow(non_snake_case)]

//! 前端

// 使用类型别名
use super::preclude::*;

// 标准库
use std::fs::File;
use std::io::{self, BufRead, Write, BufReader, Cursor};
use std::path::Path;
use std::cell::{
    RefMut, Ref, RefCell
};
