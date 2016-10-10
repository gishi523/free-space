free-space
====

![free space](https://dl.dropboxusercontent.com/u/36775496/freespace.png)

## 概要
- [1]を参考にfree space computationを実装したものです
- ステレオ視差画像からfree space(走行可能領域)を計算します

## 参考
- [1] D. Pfeiffer, U.Franke: "Towards a Global Optimal Multi-Layer Stixel Representation of Dense 3D Data", British Machine Vision Conference, August 2011

## Requirement
- OpenCVが必要です

## ビルド
```
$ git clone https://github.com/gishi523/free-space.git
$ cd free-space
$ mkdir build
$ cd build
$ cmake ../
$ make
```

## 使い方
```
./freespace left-image-format right-image-format camera.xml
```
- left-image-format
    - 左側の連番画像
- left-image-format
    - 右側の連番画像
- camera.xml
    - 計算に必要なカメラパラメータ
    - シーンに応じて設定して下さい

### 実行例
```
./freespace images/img_c0_%09d.pgm images/img_c1_%09d.pgm ../camera.xml
```

### データ
- DaimlerのGround Truth Stixel Datasetに含まれる画像およびカメラパラメータで動作を確認しています
- http://www.6d-vision.com/ground-truth-stixel-dataset

## Author
gishi523
