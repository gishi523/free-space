free-space
====

## 概要
[1]を参考にfree space computationを実装したものです
ステレオ視差画像からfree space(走行可能領域)を計算します

[1] D. Pfeiffer, U.Franke: "Towards a Global Optimal Multi-Layer Stixel Representation of Dense 3D Data", British Machine Vision Conference, August 2011

## Requirement
OpenCVが必要です

## ビルド
```
$ git clone https://github.com/gishi523/free-space.git
$ cd free-space
$ mkdir build
$ cd build
$ cmake ../
$ make
```

## 実行
./free-space left-image-format right-image-format camera.xml

## Author
gishi523