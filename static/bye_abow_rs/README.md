# 词袋模型 / bye_abow_rs : Bag of Words (BoW) in Rust.
一个Rust库，用于将图像特征描述符集合转换为“词袋”表示，以便在定位/SLAM系统中快速匹配图像。使用分层k-means聚类创建一个常见视觉特征的“词汇”。然后可以使用该词汇将新的图像或图像关键点描述符集合转换为紧凑的词袋（bow）向量。Bow向量可以非常快速地匹配，以提供图像相似性的度量。使用纯rust的image库和bye_orb_rs库取代opencv库进行orb特征提取。

**采用纯rust编写, 采用纯rust依赖**

此库主要设计用于用户提供的关键点描述符。目前支持32位二进制描述符（ORB或BRIEF是流行的例子）。然而，此库确实提供了从图像计算ORB描述符的便利函数。

## 使用示例
从一组图像创建描述符词汇并保存：
```sh
cargo run --release --example create_voc
```
输出:
```sh
词汇 = 词汇 {
    词/叶节点: 3125,
    其他节点: 780,
    层级: 5,
    分支因子: 5,
    总训练特征: 131376,
    最小词聚类大小: 1,
    最大词聚类大小: 373,
    平均词聚类大小: 42,
}
```
加载词汇，将一系列图像转换为BoW，并计算它们之间的最佳匹配：
```sh
cargo run --release --example match
```
输出:
```sh
"100.jpg"的前5个匹配：
匹配     | 分数
"100.jpg" | 1.0
"102.jpg" | 0.4220034
"101.jpg" | 0.4040035
"98.jpg"  | 0.3740036
"99.jpg"  | 0.37200385
```

## 参考
Abow受[DBoW2](https://github.com/dorian3d/DBoW2/)和[fbow](https://github.com/rmsalinas/fbow)的启发。
