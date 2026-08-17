# MAHO

## 使用言語
c++17

## コーディング規約
Google C++ Style Guideに従うこと

## ビルド方法
ninjaを使用する場合(推奨)
```bash
cmake -B build -S . -G Ninja
```
```bash
ninja -C build
```

## ディレクトリ構成
| ディレクトリ名 | 内容 |
| --- | --- |
| doc | ドキュメント |
| include | 外部に公開するヘッダー |
| src | ソース・外部に公開しないヘッダー |
| test | テスト |
