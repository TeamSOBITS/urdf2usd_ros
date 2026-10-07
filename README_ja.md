<a name="readme-top"></a>

[EN](README.md) | [JA](README_ja.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# URDF2USD with ROS Bridge

<!-- INTRODUCTION -->
## 概要

ROS 2対応のモバイルマニピュレータ用URDFを，物理駆動設定（Physics Drives）やセンサー設定を含んだ状態で，NVIDIA Isaac Sim用のUSDファイルへ変換する汎用ツールです．

**Isaac Sim 5.0〜6.1** に対応しています（[互換性表](#compatibility)を参照）．

**主な機能:**
- **ワンコマンド変換:** URDFからUSDへの変換を1行のコマンドで実行可能．
- **モバイルベース対応:** フローティングベース（浮遊ベース）およびナビゲーション用の速度制御ホイールを自動設定．
- **センサーの注入:** カメラ，LiDAR，IMUの設定をYAMLファイル経由で簡単に追加可能．
- **名前空間（Namespace）対応:** マルチロボットシミュレーションのためのROS名前空間を完全サポート．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- GETTING STARTED -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System    | Version                 |
| :-------- | :---------------------- |
| Ubuntu    | 22.04 / 24.04           |
| ROS       | 任意のROS 2ディストリビューション |
| Python    | 3.12                    |
| Isaac Sim | 5.0.0 - 6.1.0           |


> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)に参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<a name="compatibility"></a>
### 互換性

| Isaac Sim | Python | URDFインポータ                    | LiDARの既定値 | 状態               |
| :-------- | :----- | :------------------------------- | :------------ | :----------------- |
| 5.0       | 3.11   | `URDFParseAndImportFile`（従来）   | PhysX         | 本環境では未検証     |
| 5.1       | 3.11   | `URDFParseAndImportFile`（従来）   | PhysX         | 本環境では未検証     |
| 6.0       | 3.12   | `URDFImporter`（`urdf-usd-converter`） | RTX      | 想定どおり動作する見込み（6.1と同一API），本環境では未検証 |
| 6.1       | 3.12   | `URDFImporter`（`urdf-usd-converter`） | RTX      | 検証済み            |

インストールされている`isaacsim`のバージョンから，バックエンドが自動で選択されます（[isaac_version.py](utils/isaac_version.py)）．

> [!IMPORTANT]
> Isaac Sim 6.xではPhysX LiDARが廃止されました．`implementation: "physx"`を指定すると警告を出して`rtx`に切り替わり，6.xでは`rtx`が既定値になります．5.xの既定値は`physx`のままです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. **Isaac Simのインストール:** Isaac Sim本体以外の外部依存関係はありません．複雑なパス設定の問題を避けるため，[PIPとCondaを使用したインストール（英語）](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_python.html#installation-using-pip) を強く推奨します．

2. **リポジトリのクローン:**
    ```sh
    $ git clone https://github.com/TeamSOBITS/urdf2usd_ros
    ```

3. これでロボット記述ファイルの変換準備は完了です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- LAUNCH AND USAGE -->
## 実行方法

1. **URDFの準備:** xacroファイルを使用している場合は，単体のURDFファイルに変換してください．
   ```sh
   $ ros2 run xacro xacro -o output.urdf input.urdf.xacro
   ```
> [!TIP]
> Isaac SimがROSパッケージの場所を特定できず，メッシュファイルのインポートに失敗することがあります．その場合，YAMLの`ros_package_paths`でパッケージ名とディレクトリを対応付けてください（5.x・6.xの両方で有効，後述）．ROSがsourceされていれば，6.xは`ament_index`から自動で解決します．URDF内の`package://`を絶対パスに書き換える方法も使えます．

> [!WARNING]
> すべてのジョイント名，リンク名，およびメッシュファイル名は，[Isaac Simの命名規則（英語）](https://docs.omniverse.nvidia.com/usd/code-docs/usd-exchange-sdk/latest/api/group__names.html#group__names_1autotoc_md9) に厳密に従う必要があります（例：ハイフン `-` の使用は避ける）．これに従わない場合，変換に失敗する可能性があります．


2. **ロボットの設定:** テンプレートファイル [robot_template.yaml](config/robot_template.yaml) をコピーし，対象ロボットのジョイント名やセンサーリンクに合わせて内容を編集してください．
> [!NOTE]
> 新しいYAML設定ファイルは [config](config) フォルダ内に配置し，ファイル名にはスペースを含めないでください．

   `ros_package_paths`は，`package://`を解決するためのパッケージ名と絶対パスの対応表です．
   ```yaml
   ros_package_paths:
     sobit_home_description: /path/to/sobit_home_description
   ```
   5.xでは`package://`を書き換えたURDFの一時コピーを使用し（変換後に削除），6.xではインポータの`ros_package_paths`に渡されます．

3. **環境の確認:** Isaac SimのPython環境が有効になっていることを確認してください（Condaを使用したインストール方法に従った場合，この手順は自動的に行われます）．

4. **スクリプトディレクトリへの移動:**
   ```sh
   $ cd urdf2usd_ros/scripts/
   ```

5. **URDFからUSDへの変換:**
   設定ファイル名（拡張子なし）を引数に指定して，変換スクリプトを実行します．
   ```sh
   $ python3 urdf2usd_ros.py --robot {作成したYAMLファイル名}
   # 例: python3 urdf2usd_ros.py --robot sobit_light
   ```

6. **結果:** 設定済みのUSDファイルが，YAMLファイル内で指定した出力ディレクトリに生成されます．

> [!NOTE]
> **Isaac Sim 6.xの出力構成:** 6.xのインポータは単一ファイルではなくディレクトリ（`<name>/<robot>.usda`，`payloads/`，`Textures/`）を出力します．`files_path.usd`は従来どおり指定したファイルとして生成され，パッケージはその隣の`<USDファイル名>/`に移動し，`<name>.usd`はそのパッケージを参照する薄いラッパーになります（`Physics`バリアントセットの`physx`を選択し，センサーとOmniGraphもここに保存されます）．ラッパーとパッケージのディレクトリは一緒に保管してください．`files_path.usd`がディレクトリの場合は，インポータのメイン`.usda`がそのまま使われます．

> [!NOTE]
> **ROS 2ライブラリ:** ROS 2がsourceされていない場合，スクリプトはIsaac Sim 6.xに同梱のROS 2 Jazzyライブラリ（`ROS_DISTRO=jazzy`，`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，`LD_LIBRARY_PATH`に`isaacsim.ros2.core/jazzy/lib`を追加）で自身を再実行します．`OMNI_KIT_ACCEPT_EULA`が未設定なら`yes`を設定します．ステージを自分で読み込む場合は，ROS 2拡張を有効化した後に`app.update()`を数回実行してからステージを開いてください（6.1では，有効化直後に開くと`omni.graph.core`がクラッシュしました）．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Isaac Simでの可視化

複雑なシミュレーションを実行する前に，生成されたアセットを可視化してテストすることをお勧めします．

1. Isaac Simを起動します．
   ```sh
   $ isaacsim
   ```

2. 新しく生成されたUSDファイルを開きます．
3. **再生ボタン (▶)** をクリックして，物理シミュレーションを開始します．
4. ロボットの関節を手動で操作し，正しく可動することを確認します．
5. ロボットが目標位置に到達できない，または挙動が不安定な場合は，YAML設定ファイル内の `stiffness`（剛性）と `damping`（減衰）の値を調整し，USDを再生成してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### テスト

ロボットを変換し，結果（アーティキュレーション，自由度数，ドライブゲイン，センサー，OmniGraphノード型，120フレームの物理演算，ROS 2コンテキスト）を検証します．Isaac SimのPython環境で実行してください．
```sh
$ python3 tests/convert_and_check.py --robot sobit_home
# Isaac Lab venv: cd IsaacLab && uv run --no-sync python /path/to/urdf2usd_ros/tests/convert_and_check.py --robot sobit_home
```
PASS/FAILの表を表示し，失敗があれば非ゼロで終了します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### ロボット側の既知の問題

- **SOBIT HOMEの`plate_cover_joint`:** 親が質量のない`base_footprint`のため，`plate_cover_link`（1.5 kg）が2つ目のアーティキュレーションルートとなり，ワールドに固定されます（120フレームのheadless実行で，カバーはz=0.496のままベースは0.309から0.413へ移動）．`sobit_home_description`側で`plate_cover_joint`の親を`plate_middle_link`に変更し，`xyz="0 0 0.2249"`（0.496258 − 0.271358）としてください．`base_link`にはinertialがないため，`base_link`への変更では解決しません．
- **TFに含まれない質量のないフレーム:** `base_footprint`，`base_link`，`lidar_merged_laser`はアーティキュレーションに含まれないため，生成されたTFグラフはこれらを配信しません．`robot_state_publisher`を併用する（ROSの通常構成）か，静的変換を追加してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- MILESTONE -->
## マイルストーン

- [ ] 複数のモバイルベースコントローラのサポート
- [ ] スワーブ駆動ベースのサポート（`sobit_home.yaml`では`mobile_base`を無効化．差動コントローラのグラフは適用不可）（TODO `swerve`）
- [ ] YAMLパラメータの最小化（URDFからの自動検出）
- [ ] TF Staticトピックの配信
- [ ] カスタムQoS設定のサポート
- [ ] TF名前空間（Namespace）のサポート
- [x] 差動ドライブコントローラ（Differential Drive Controller）のサポート
- [x] ROS 2 Bridgeの自動統合
- [x] Isaac Sim 6.0 / 6.1への対応

現時点のバッグや新規機能の依頼を確認するために[Issueページ][issues-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- ACKNOWLEDGMENTS -->
## 参考文献

* [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
* [Extension: URDF Importer](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.asset.importer.urdf/docs/index.html)
* [API: UsdGeomCamera](https://docs.omniverse.nvidia.com/kit/docs/usdrt.scenegraph/7.6.2/api/classusdrt_1_1_usd_geom_camera.html)
* [Sensor: PhysX SDK Lidar](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_physx_lidar.html)
* [Sensor: RTX Lidar](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_rtx_lidar.html)
* [Sensor: IMU](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_physics_imu.html)
* [Extension: ROS 2 Bridge](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.ros2.bridge/docs/index.html)
* [Extension: Physics Sensor Simulation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.physics/docs/index.html)
* [Extension: Wheeled Robots](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.robot.wheeled_robots/docs/index.html)
* [Extension: Core OmniGraph Nodes](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.nodes/docs/index.html)
* [Concept: Action Graph](https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/concepts/ActionGraph.html)
* [Reference: OmniGraph Nodes](https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes_core/latest/Overview.html)
* [URDF Specification](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
* [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [ROS 2 Control](https://control.ros.org/jazzy/index.html)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/urdf2usd_ros.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/urdf2usd_ros/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/urdf2usd_ros.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/urdf2usd_ros/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/urdf2usd_ros.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/urdf2usd_ros/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/urdf2usd_ros.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/urdf2usd_ros/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/urdf2usd_ros.svg?style=for-the-badge
[license-url]: LICENSE
