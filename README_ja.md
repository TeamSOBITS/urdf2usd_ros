[EN](README.md) | [JA](README_ja.md)

# URDF2USD with ROS Bridge

<!-- INTRODUCTION -->
## 概要

ROS 2対応のモバイルマニピュレータ用URDFを，物理駆動設定（Physics Drives）やセンサー設定を含んだ状態で，NVIDIA Isaac Sim用のUSDファイルへ変換する汎用ツールです．

**Isaac Sim 5.0以降** に対応しています.

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
| Isaac Sim | 5.0.0+                  |


> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)に参照してください．

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
> Isaac SimがROSパッケージの場所を特定できず，メッシュファイルのインポートに失敗することがあります．その場合，URDF内のファイルパス（`package://`）を絶対パスに書き換えることを推奨します．

> [!WARNING]
> すべてのジョイント名，リンク名，およびメッシュファイル名は，[Isaac Simの命名規則（英語）](https://docs.omniverse.nvidia.com/usd/code-docs/usd-exchange-sdk/latest/api/group__names.html#group__names_1autotoc_md9) に厳密に従う必要があります（例：ハイフン `-` の使用は避ける）．これに従わない場合，変換に失敗する可能性があります．


2. **ロボットの設定:** テンプレートファイル [robot_template.yaml](config/robot_template.yaml) をコピーし，対象ロボットのジョイント名やセンサーリンクに合わせて内容を編集してください．
> [!NOTE]
> 新しいYAML設定ファイルは [config](config) フォルダ内に配置し，ファイル名にはスペースを含めないでください．

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


<!-- MILESTONE -->
## マイルストーン

- [ ] 複数のモバイルベースコントローラのサポート
- [ ] YAMLパラメータの最小化（URDFからの自動検出）
- [ ] TF Staticトピックの配信
- [ ] カスタムQoS設定のサポート
- [ ] TF名前空間（Namespace）のサポート
- [x] 差動ドライブコントローラ（Differential Drive Controller）のサポート
- [x] ROS 2 Bridgeの自動統合

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
