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
     my_description: /path/to/my_description
   ```
   URDFが使用するすべての`package://`（メッシュやテクスチャ．センサーやサードパーティの記述パッケージも含む）は，YAMLに記載するか検出可能である必要があります．YAMLにないパッケージは`ament_index`，次に`$AMENT_PREFIX_PATH` / `$COLCON_PREFIX_PATH` / `$ROS_PACKAGE_PATH`（`share/<pkg>`）から探索されるため，ROSをsourceしなくてもこれらの環境変数を設定すれば十分です．解決できないパッケージごとに`WARNING: Unresolved package://<pkg> (N meshes: ...)`を表示してインポートを中止します（`import: {allow_missing_meshes: true}`で続行可能）．
   5.xでは`package://`を書き換えたURDFの一時コピーを使用し（変換後に削除），6.xではインポータの`ros_package_paths`に渡されます．

   **質量のない親リンク:** Isaac 6のインポータは，親リンクに`<inertial>`がない固定ジョイントをワールドに固定し，その子リンクを別のアーティキュレーションルートにしてしまいます．そのため本ツールは，子リンクが質量を持つ場合，そのジョイントの親を，固定結合でつながった最も近い質量を持つ祖先（なければ質量のないクラスタ内で最初の質量を持つリンク）に付け替えます．ジョイントのoriginを合成するため，姿勢とTFフレーム名は変わりません．付け替えたジョイントごとに1行ログを出力します．YAMLに`import: {fix_massless_parents: false}`を指定すると無効になります．すべての対応バージョンで動作し，元URDFの隣に一時コピーを作成して処理します．

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


### ロボットディスクリプタ

YAMLに`robot_descriptor: <robot_id>`を書くと，ロボット固有の情報を共有の`<robot_id>.robot.yaml`（[sobits_robot_descriptor](../sobits_robot_descriptor)）から取得します：`ros2.namespace`，`ros2.topic_joint_states`，`ros2.controllers`，`files_path.urdf`（プレースホルダーのときのみ`<description share>/<urdf.urdf>`），および各`sensors.<name>`の`type`，`parent_link`，`frame_id`（オプティカルフレーム），`image_width/height`，`update_rate`，`rgb`/`depth`/`pcl`とLiDAR/IMUのトピックです．YAMLに明示した値が常に優先されます．`sensors`はディスクリプタのcamera/lidar/imu名をキーとし，Isaac専用のパラメータ（アパーチャ，クリッピング，回転，LiDARプロファイル，IMUフラグ）のみを記述します．未知の名前はディスクリプタのセンサー一覧を表示して中断し，`requires`で除外されたセンサー（例：`--xacro-arg head_cam_type=realsense`のOrbbec IMU）は通知を出して除外されます．キーのない設定は従来どおり動作します．

- **解決順序：** `--descriptor ID|PATH`（なければ`robot_descriptor`キー）．IDは，既存のファイルパス，`$SOBITS_ROBOT_DESCRIPTOR_PATH`（コロン区切りのファイル／ディレクトリ），`ament_index`，`$AMENT_PREFIX_PATH` / `$COLCON_PREFIX_PATH`の順に探します．
- **ローダーのimport（ROS不要）：** venvにインストール（`uv pip install -e <src>/sobits_robot_descriptor`），または`$SOBITS_ROBOT_DESCRIPTOR_PYTHONPATH`，amentプレフィックス，隣のチェックアウト`../sobits_robot_descriptor`．
- **バリアント：** `--xacro-arg K=V`（複数指定可）でディスクリプタのバリアント（例：`head_cam_type`）を選択します．
- **コントローラ：** `ros2.controllers.<name>.topic`は関節状態サブスクライバが使うコントローラ名（`body_position_controller`など．完全なトピック名ではありません）で，`<name>`はグループ名（`head`，`body`，`arm_left`，...，`wheel_drive`），`type`はコントローラの種類です．

```sh
$ SOBITS_ROBOT_DESCRIPTOR_PATH=/path/to/sobit_home_description/config python3 scripts/urdf2usd_ros.py --robot sobit_home
```
`config/sobit_home.yaml`が基準です．SOBIT HOMEの`tests/convert_and_check.py`は24/24でパスします（ヘッドカメラ，両ハンドカメラ，統合LiDAR，IMUを含む）． `config/sobit_light.yaml`はSOBIT LIGHT用の同形式の設定です（Kachaka差動駆動ベース，`ros2.mobile_base`グラフ，URDFは`enable_gz:=True`で生成し`file://`メッシュパスを`package://`に置換）．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 関節ドライブゲイン

YAMLのゲインはSI単位です（回転関節: N·m/rad，N·m·s/rad，直動関節: N/m，N·s/m）．USDは回転ゲインを度単位で保持するため，書き込み時に ×π/180 で変換します．関節ごとにSI値，USD値，ゲインから求まる固有振動数が出力されます．

- **`default_drive.mode: auto`（推奨）:** 関節ごとに `k = max(I_max·(2πf)², τ_g,max / e_max)`，`d = 2ζ·sqrt(k·I_max)` を計算します．`I_max` と `τ_g,max` は子リンク以下全体の慣性と重力トルクの上限（URDFのinertialから算出，姿勢非依存），`f`，`ζ`，`e_max` は `natural_frequency`（Hz），`damping_ratio`，`max_sag_deg` / `max_sag_m` です．手首や指のような軽い関節には肩よりずっと小さいゲインが設定されます．
- **速度制御関節:** `joints:` で `stiffness: 0.0` を指定すると剛性0のままになり，`damping`（省略時は `default_drive.velocity_damping`）が使われます．
- **手動:** `mode` を省略して `default_drive.stiffness` / `damping` を指定すると全関節に同じ値を使います．個別の関節は `joints:` で上書きでき，autoモードでも上書きが優先されます．
- **質量のないリンク:** `<inertial>` のないリンクにはPhysXが既定の質量（形状なしは1 kg，visualありは密度から算出）を割り当て，警告を表示します．URDFにinertialを追加してください．

### 初期姿勢

Isaacはすべての関節をURDFのゼロ姿勢で読み込むため，ロボットの一部が床下に入ることがあります．初期姿勢はURDFの `<ros2_control>`（`<state_interface name="position">` の `initial_value`，rad または m）から取得し，YAMLの任意のトップレベルキー `initial_pose: {関節名: 値}`（SI単位）で上書き・追加できます．回転関節の値はUSD用に度へ変換され，ドライブ目標とジョイント状態（`PhysicsJointStateAPI`）の両方に書き込まれるため，ロボットはその姿勢で開始し保持します．同じ値は `newton:angular:position` / `newton:linear:position` にも書き込まれ，NewtonのUSDインポータはここから開始位置を読みます．関節リミット（`physics:lowerLimit`/`upperLimit`）を超える値はリミットにクランプされ通知が出力されます（例: URDFの `initial_value` -1.571 radは-90°のリミットをわずかに超えます）．速度制御の関節はスキップされ，存在しない関節には警告が出ます．

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
5. ロボットが目標位置に到達できない，または挙動が不安定な場合は，ドライブゲイン（関節ドライブゲインの節を参照）を調整し，USDを再生成してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### テスト

ロボットを変換し，結果（アーティキュレーション，自由度数，ドライブゲイン，センサー，OmniGraphノード型，120フレームの物理演算，ROS 2コンテキスト）を検証します．Isaac SimのPython環境で実行してください．
```sh
$ python3 tests/convert_and_check.py --robot {YOUR_ROBOT_YAML_FILE_NAME} [--urdf FILE] [--usd FILE] [--package-path NAME=PATH] [--descriptor ID|PATH] [--xacro-arg K=V] [--skip-step-test]
# Isaac Lab venv: cd IsaacLab && uv run --no-sync python /path/to/urdf2usd_ros/tests/convert_and_check.py --robot ...
```
PASS/FAILの表を表示し，失敗があれば非ゼロで終了します．
動的チェックは（USDには保存されない）セッションレイヤー上の地面で実行します：`hold pose at zero target`，`step tracking`（位置駆動の各自由度を0.3 rad / 0.1 mだけ動かし，0.02 rad / 0.01 m以内で追従，他の自由度は0.03 rad以内），`mimic joints coupled`（URDFの`<mimic>`従動関節が主関節に20%以内で追従），`base stays put`（保持中のルート移動0.02 m未満，ステップ全体で0.10 m未満）．
`--skip-step-test`でステップとmimicのチェックを省略できます（約41x180フレーム分）．
マシン固有のパスはコミットするYAMLに含めず，`--urdf`，`--usd`，`--package-path`（複数指定可，`scripts/urdf2usd_ros.py`でも可）で渡すか，Git管理外の`config/{robot}.local.yaml`で上書きしてください（同じ構造で`files_path`や`ros_package_paths`のみ記述でき，コミット済みYAMLにマージされます）．`scripts/urdf2usd_ros.py`は`--config PATH`も受け付けます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### MJCFの出力

変換したロボットは，同じドライブゲイン，mimic結合，初期姿勢をもつMuJoCoのMJCFとしても出力できます．USDは[Newton](https://github.com/newton-physics/newton)のUSDインポータで読み込み，そのMuJoCoソルバで出力するため，Isaac Simは不要です．Python環境に `newton`（`newton_usd_schemas`，`warp` を含む）と `mujoco` が必要です（Isaac Labのvenvには両方あります）．

```sh
# 変換の直後に出力（scripts/usd2mjcf.py を別プロセスで実行）
$ python3 scripts/urdf2usd_ros.py --robot sobit_home --mjcf [PATH]
# 既存のUSDから出力
$ python3 scripts/usd2mjcf.py --robot sobit_home [--usd FILE] [--urdf FILE] [--mjcf FILE] [--ground]
# Isaac Lab venv: cd IsaacLab && uv run --no-sync python /path/to/urdf2usd_ros/scripts/usd2mjcf.py --robot ...
```
既定の出力先はUSDのパスの拡張子を `.xml` にしたものです．`files_path.usd` と初期姿勢（URDFの `ros2_control` + YAMLの `initial_pose`）は変換と同じYAMLから取得します．Python API: `utils.mjcf_export.export_mjcf(usd_path, mjcf_path, initial_pose=None, ground=False, keep_prims=None)` はボディ・関節・アクチュエータ・メッシュ・等式拘束の数を返します．NewtonのUSDスキーマを先に登録する必要があるため，USDステージを開く前に `utils.mjcf_export` をimportしてください（順序が逆だとmimic関節が失われるため，出力はエラーで停止します）．

- **読み込み前に除去するもの**（メモリ上のみ，USDは変更しません）: 型が `OmniLidar`，`IsaacImuSensor` のプリム，`*Graph` のプリム（ROS 2のOmniGraph），`http(s)://` や `omniverse:` のアセットを参照するプリム（RTX lidarのプロファイル．残すとコンポジションエラーになります）．カメラは残します（Newtonは無視します）．その他は `--keep-prim PATH` で残せます．
- **内容:** ボディと関節はURDFのリンク名・関節名のまま（浮遊ベースの関節は `root`），位置ドライブごとに `general` の位置アクチュエータ（`kp`/`kv` はSI単位でUSDと同じ値），`stiffness: 0` の関節は速度アクチュエータ，URDFの `<mimic>` は `<equality><joint>`，`<compiler angle="radian">`，`home` キーフレーム（`qpos` は初期姿勢で浮遊ベースは記述されたルート位置，`ctrl` は位置目標）．
- **床:** 地面は出力しません．利用側で追加してください（単体テスト用に `--ground` で残せます）．出力時は地面ありでモデルを構築するため，Newtonが生成する衝突マスクは既定の床や物体（`contype=conaffinity=1`）とは接触し，ロボット自身とは接触しません（USDと同じく自己衝突なし）．地面なしで構築すると全ジオメトリが `contype=conaffinity=0` になります．
- **見た目:** visualジオメトリはgroup 1で，USDでバインドされたマテリアルの `diffuseColor` から `rgba` を設定します（`GeomSubset` のバインドを含む）．コライダーはgroup 3（ビューアでは既定で非表示）．テクスチャは引き継ぎません．
- **メッシュはインライン**（XML内の `vertex`/`face`）のため，ファイル単体で完結しますがサイズが大きくなります: SOBIT HOMEで約54 MB，SOBIT LIGHTで約27 MB．
- **MuJoCoのバージョン:** MuJoCo 3.0.0，3.8.1，3.12で読み込めることを確認済みです．

`tests/check_mjcf.py` は出力したファイルを素のMuJoCoで検証します．
```sh
$ python3 tests/check_mjcf.py --robot sobit_home [--usd FILE] [--mjcf FILE]
```
USDの関節とドライブに対する `nq`/`nv`/`nu` の数，`utils.drive_gains` に対するアクチュエータゲイン（SI，1%以内），URDFに対するmimic等式拘束，`home` キーフレームが初期姿勢と一致すること，床の上で `home` から2秒間の保持（床がなければ追加．全関節0.02 rad以内，ベース0.02 m以内），visualジオメトリの `rgba` とコライダーのgroup 3を確認します．SOBIT HOMEとSOBIT LIGHTは12/12でパスします．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Isaac Lab

`lab/` は変換したロボットを[Isaac Lab](https://github.com/isaac-sim/IsaacLab) 3.0上のPhysXまたはNewton（MuJoCo-Warp）で，必要ならワールドUSDの中で動かします．姿勢保持・追従のテストとしても使えます．必要なもの: Isaac Lab 3.0のチェックアウトとそのuv venv（`isaaclab_physx`，`isaaclab_newton`，`isaaclab_tasks`），`robot_descriptor` をもつ設定YAML，変換済みの `files_path.usd`．

```sh
$ cd <IsaacLab>
$ export SOBITS_ROBOT_DESCRIPTOR_PATH=/path/to/sobit_home_description/config:/path/to/sobit_light_description/config
$ uv run --no-sync python /path/to/urdf2usd_ros/lab/run.py --robot sobit_home [--backend physx|newton] \
    [--world WORLD.usda] [--spawn X Y Z YAW_DEG] [--num-envs N] [--hold] [--wave] [--wave-group NAME] \
    [--steps N] [--keep-open] [--device cpu|cuda] [--viz kit|newton_gl]
# 例: gz-usdで出力したRoboCup@Homeアリーナ，GUIあり
$ uv run --no-sync python /path/to/urdf2usd_ros/lab/run.py --robot sobit_light --backend newton \
    --world /path/to/sobits_gazebo_worlds/export/usd/rcw2026_arena.usda --spawn -2.5 -2.5 0.002 90 --hold --wave --viz kit --keep-open
```
`--viz` を付けなければヘッドレスで実行します．フェーズは順に各 `--steps` 環境ステップ（50 Hz，既定250 = 5秒）実行します: `--hold`（既定）は初期姿勢を保持し，`--wave` は1つのグループを `--wave-amp`（0.25）・`--wave-hz`（0.5）で動かして追従誤差を測ります．シミュレーション1秒ごとにルートリンクの高さ，位置制御関節の max |q − q_init|，追従誤差，NaNチェックを表示し，最後に `[lab] SUMMARY {json}` を出力します．NaN，`--max-hold-dev`（0.05）を超える保持偏差，`--max-track-err`（0.05）を超える追従誤差のいずれかで非ゼロ終了します．

- **前処理ファイル**（`lab/prepare.py`．pxrのみで別プロセス実行し，`output/lab/` にキャッシュ）: `output/lab/<robot>/<robot>.usda` はOmniGraph，RTX lidar，IMUのプリムを除き，PhysicsSceneを無効化したロボットUSDです（`/physicsScene` はIsaac Labが持ちます）．`meta.json` には関節，グループ，初期姿勢，ルートのオフセットが入ります．ワールドは `output/lab/worlds/` にオーバーレイレイヤを作ります．入力（USD，URDF，設定，ディスクリプタ，`prepare.py`）が変わると再生成します．中身の確認は `lab/prepare.py --robot NAME [--world W]` で行えます．
- **ディスクリプタから導出するもの:** ディスクリプタのグループ（`joints` + `uncommanded_joints`）と `mobile_base.controllers` の各エントリごとに `ImplicitActuatorCfg` を1つ作ります．関節名は完全一致で，`stiffness`/`damping` はUSDのドライブ値（変換時のゲイン．両バックエンドで同じ値）を使います．どのグループにも属さないUSDの関節（Kachakaの `docking_joint` や車輪など `excluded_joints` の関節）は `excluded` グループとしてUSDのドライブのまま扱います．アクションは `kind: position` のグループ（モバイルベースのコントローラを除く）の関節に対する初期姿勢からの位置オフセットです．`kind: velocity` のグループには速度目標0を与え，それ以外は初期目標を保持します．waveのグループは最初の `ee[].control.group`，なければ最初の位置グループです．スポーン位置は `base_frame` を `--spawn`（既定 `0 0 0.002 0`）に置き，USDから読んだ `base_frame` → ルートリンクのオフセットを加えます（Isaac Labはルートリンクの位置を指定します．SOBIT HOMEでは `base_footprint` の0.27 m上）．
- **初期姿勢:** `utils.initial_pose.initial_pose()` に変換時と同じ関節リミットでのクランプを適用した値，つまり変換器がUSDに書いた値です．姿勢にない関節はUSDのドライブ目標，速度制御関節は0から始まります．USDのドライブ目標と異なる場合は注意を表示します（再変換してください）．
- **バックエンド:** `--backend physx` は `isaacsim_physx` プリセット（Kit PhysX），`--backend newton` は `newton_mjwarp` プリセット（MuJoCo-Warp，`implicitfast`，pyramidal cone，接触容量 `--nconmax 4096` / `--njmax 16384`．約650接触の家具ありアリーナに合わせた値）を選びます．環境はDirectワークフローの環境（`lab/env.py`，`LabEnvCfg.setup(robot_meta, world_meta, spawn)`）で，プリセットは `isaaclab_tasks.utils.hydra.resolve_presets` で解決します．
- **ワールドの回避策**（gz-usdで出力したワールド向け．それぞれ `lab/prepare.py` の個別の手順で，実行開始時に一覧表示します）: ワールド自身のPhysicsSceneを無効化（常に）．マテリアルの摩擦係数を `--max-friction` にクランプ（Newtonでは既定1.0，PhysXでは無効．μ = 50/100のようなGazebo ODEの値ではMuJoCo-Warpの接触が張り付きます）．固定ジョイントだけで結合された自由な複数ボディのモデル（かごなど）には，Newtonでは `ArticulationRootAPI` を付与（ないとNewtonのインポータが受け付けません）．Newtonで `--num-envs > 1` のときはワールドを環境ごとに複製し（`--world-per-env` で強制），間隔はワールドのxy範囲 + 1 mです（Newtonのグローバルワールドはボディを持てないため）．`--world` がなければ地面を使います．

`tests/check_lab.py` はロボット × バックエンドのすべての組み合わせを `--hold --wave` で別プロセス実行し，表を表示します（ログは `output/lab/check/`）．`--vram-limit-mib` を超えるGPUメモリ使用でその実行を停止します．
```sh
$ uv run --no-sync python /path/to/urdf2usd_ros/tests/check_lab.py --robots sobit_home sobit_light \
    --world /path/to/rcw2026_arena.usda --spawn -2.5 -2.5 0.002 90 [--backends physx newton] [--device cpu]
```
RTX 3080 Ti（PhysXはGPU），各1環境で4実行に約100秒かかります．RCW2026アリーナ（`--spawn -2.5 -2.5 0.002 90`，保持5秒 + wave 5秒．hold = 位置制御関節の max |q − q_init| [rad または m]，track = 0.5秒以降のwave追従誤差の最大値）での結果:

| robot | backend | hold | track | ルートリンク z 終了時 [m] | env steps/s |
|---|---|---|---|---|---|
| sobit_home | physx | 0.0145 | 0.0221 | 0.2714 | 15 |
| sobit_home | newton | 0.0158 | 0.0201 | 0.2711 | 209 |
| sobit_light | physx | 0.0105 | 0.0248 | 0.0000 | 23 |
| sobit_light | newton | 0.0104 | 0.0222 | 0.0000 | 240 |

地面のみの場合と，Newtonで `--num-envs 2`（環境ごとのアリーナ）の場合も，SOBIT HOMEの値は2e-4以内で一致します．

既知の問題:
- **GPUを共有するとGPU PhysXが遅い:** デスクトップや他のGPU処理と同じカードでは，GPU PhysXでSOBIT HOMEが約15 env steps/s（実時間の0.3倍）でした．`--device cpu` ではPhysXが140〜170 env steps/sで同じ結果になります．NewtonはGPUで約200 steps/sです．
- **steps/s** は最初のシミュレーション1秒（ウォームアップ，NewtonのCUDAグラフ取得）を含みません．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### ロボット側の既知の問題

- **質量のない親リンク下の固定ジョイント:** 自動的に処理されます（上記の`import.fix_massless_parents`を参照）．無効にする場合は，このようなジョイントの親を慣性を持つ最も近いリンクに変更する（originも調整），または親リンクにinertialを追加してください．そうしないと子リンクはワールドに固定された別のアーティキュレーションルートになります．
- **TFに含まれない質量のないフレーム:** 質量のないフレームはアーティキュレーションに含まれないため，生成されたTFグラフはこれらを配信しません．`robot_state_publisher`を併用する（ROSの通常構成）か，静的変換を追加してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- MILESTONE -->
## マイルストーン

- [ ] 複数のモバイルベースコントローラのサポート
- [ ] スワーブ駆動ベースのサポート（差動コントローラのグラフは適用不可．`mobile_base`は無効のままにする）（TODO `swerve`）
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
