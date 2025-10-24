## 現状仕様の整理

- **ランタイム構成**
  - `src/ros/runtime/pointcloud_compressor.cpp` が外部API(`RuntimeAPI.hpp`)に応じて圧縮処理を実行。
  - `RuntimeHelpers`、`CompressionArtifacts`、`TempFileManager` などのヘルパが辞書・インデックス生成やテンポラリ管理を担当。
  - HDF5入出力は `pointcloud_compressor/io/HDF5IO` に委譲し、圧縮結果のメタデータを `CompressedMapData` 構造体へマッピング。
- **コアロジック**
  - `PointCloudCompressor` が点群読み込み、ボクセル化、辞書構築、エンコード、HDF5書き出しを包括的に処理。
  - OpenMPを用いたボクセル分割(`VoxelProcessor`)など、一部演算が並列化されている。
- **エントリポイント**
  - CLI (`src/apps/cli_main.cpp`) はコマンドラインオプションを解析し、RuntimeAPI経由で圧縮。
  - ROS2ノード (`src/ros/pointcloud_compressor_node.cpp`) はROSパラメータを取得し、RuntimeAPIを介して単発配信を実施。Marker配信など可視化にも対応。
- **設定・プリセット**
  - デフォルト設定は `config/pointcloud_compressor_params.yaml`。プリセットは `pointcloud_compressor_preset.launch.py` 内の内蔵辞書で管理。
- **テスト**
  - `colcon test` 配下に多数のgtestが存在。Runtime関連の単体テスト (`test_runtime_*`) でヘルパやHDF5連携を検証。
  - 一部既存テスト（PlyIO等）は失敗状態にあり、既知課題。

## 改良が必要な点・検討事項

- **コード組織のさらなる明確化**
  - Runtime層が`buildSuccessReport`/`maybeWriteRawGrid`等でまだ大きい。レポート生成とファイル書き込みの責務分離を進める。
  - CLI・ROSノード双方で利用する共通設定パーサが無く、オプションやパラメータの重複が散在。
- **例外/エラー処理の一貫性**
  - RuntimeAPIのエラーメッセージ連結は文字列追記のみ。エラー種別やコードを定義し、クライアント側で扱いやすくする。
  - HDF5書き込みやファイルIO失敗時のリカバリ/リトライ戦略が無い。ログレベル制御も限定的。
- **パフォーマンス/メモリ最適化**
  - 大規模点群での辞書・インデックスbuffer再構築を抑えるコピー削減が今後も必要（move利用やspan導入を検討）。
  - Occupancyマスク生成も用途に応じてlazy化できる余地がある。
- **テスト体系の強化**
  - 失敗している既存テストの復旧。特にPlyIO/PointCloudIO関連の仕様変更点を再確認し、期待値更新または実装修正を行う。
  - 大規模データや異常系（ファイル破損、HDF5互換性）の自動テスト追加。
- **設定/プリセット管理の改善**
  - YAMLとプリセット辞書の同期手段がない。設定のスキーマ化や共通ローダーの導入でアップデート時の齟齬を防止。
  - CLI/ROSで異なるデフォルト値が存在しうるため、共通のConfig層を設計する。
- **ドキュメント充実**
  - リファクタリング方針、設定項目、テスト実行手順をREADMEから派生した技術文書として整備する。
