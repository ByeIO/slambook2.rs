# ORB（定向FAST和旋转BRIEF）关键点与Rust
[Rublee E, Rabaud V, Konolige K, et al. ORB: An efficient alternative to SIFT or SURF[C]//2011 International conference on computer vision. Ieee, 2011: 2564-2571.]
使用Rust和图像库实现定向FAST和旋转BRIEF描述符。

**ORB** 关键点
![ORB Keypoints](assets/out.png)

```rust
use image;
use orbrs;

fn test() {
    let mut img1 = image::open("example/a.png").unwrap();
    let mut img2 = image::open("example/b.png").unwrap();

    let n_keypoints = 50;

    let img1_keypoints = orbrs::orb::orb(&mut img, n_keypoints).unwrap();
    let img2_keypoints = orbrs::orb::orb(&mut img, n_keypoints).unwrap();

    let pair_indices = orbrs::orb::match_brief(&img1_keypoints, &img2_keypoints);
}
```

**FAST** 关键点
![FAST Keypoints](assets/fast.png)

提取ORB特征并绘制关键点的示例：
```rust
use image;
use orbrs;

fn test() {
    let mut img = image::open("example/test.jpg").unwrap();

    let fast_keypoints = orbrs::fast::fast(&img, Some(fast::FastType::TYPE_9_16), None).unwrap();

    // 在图像上绘制关键点
    
    orbrs::fast::draw_moments(&mut img.to_rgb(), &fast_keypoints);
    img.save_with_format("example/fast_output.png", image::ImageFormat::Png);
}
```

## 例程
```sh
cargo run --example demo
```