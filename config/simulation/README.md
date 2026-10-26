# 走行実験

ROS 2 Jazzy + Gazebo Harmonic。実機ドライバは起動しない。
依存はpackage.xmlのAPTパッケージを使用する。地図のオフライン処理のみOpen3D/Pillow/PyYAMLが必要。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch oit_deliverybot_sim experiment.launch.py scenario:=crossing method:=dwb repeats:=3
```

- `scenario`：`clear`、`static`、`crossing`、`following`、`oncoming`、`stop_go`、`speed_change`。カンマ区切り／`all`も可。
- `method`：`dwb`、`mppi`、`mppi_pos`、`mppi_cv`、`mppi_age`。カンマ区切りも可。
- `gui:=true`でGazeboを表示。初期位置・目標はenvironmentのYAMLで指定する。
- シナリオ内の障害物位置はworld/map座標。
- `domain`は既定71。別端末から見るときも同じROS_DOMAIN_IDを設定する。

結果は`~/ros2_ws/experiments/simulation/batch_…/`。試行ごとにbag(MCAP)、samples.csv、起動ログを保存する。
全試行のresults.csvとcomparison.pngも生成する。成功・衝突・時間切れ・走行失敗・基盤異常・取消を区別する。
基盤異常や記録された衝突を削除して成功率を計算しない。

**学校データは非公開です。** 学校の地図・rosbag・点群・メッシュ・生成済みenvironment YAMLは本リポジトリに含みません。
同梱の`corridor_map.pgm`は人工の平坦通路地図で、学校地図ではありません。
`prepare_sim_school.py`は手元の2D地図を押し出す変換処理だけを提供し、入力データは配布しません。
`prepare_sim_mesh.py`も入力点群を別途用意する必要があります。
学校データを使う例（利用権限のある入力から生成済みのファイルが必要。公開コードの通路実験には不要）：

```bash
ros2 launch oit_deliverybot_sim experiment.launch.py \
  environment:="$HOME/ros2_ws/experiments/simulation/school/environment.yaml"
```

学校の既定環境は、同じGLIM出力からoit_map_toolsで作った2D地図を高さ1.5mに押し出した平面近似。
床の凹凸・建物の実際の高さ・高さによる遮蔽を再現したものではない。元の3Dメッシュもenvironment YAMLで指定可能。
地図の単位とXY原点は変えない。初期位置は車体が重ならない既知の自由領域から人が指定する。

LiDARは720×32点、10Hzの理想モデル。反射強度・欠測・走査中の動きは未モデル化。
移動物は重力を切った剛体円柱に速度を与える。人の行動や接触後の物理反応のモデルではない。
採点用の正解位置は`/evaluation/`へ配信し、AMCL・Nav2へ入力しない。
接近距離は既知の移動物との平面外形間距離。任意の静的地図全体への厳密な最短距離ではない。

独自処理は`/sim/lidar_points`、`/sim/scan`、`/sim/odom`、目的地・経路を入力にできる。
速度は`/sim/cmd_vel_raw`へ出し、共通smootherが`/sim/cmd_vel`へ出す。制御器は一つずつ有効にする。

## MPPI 3条件（第1段階）

```bash
ros2 launch oit_deliverybot_sim experiment.launch.py \
  scenario:=all method:=mppi,mppi_pos,mppi_cv seed:=1 seed_set:=development
```

`mppi`=C0、`mppi_pos`=C1（追跡位置を静止とする）、`mppi_cv`=C2（等速予測）。
C1/C2は同じ追加critic・重み20・power 1・余裕0.25mを使い、`use_velocity`だけを変える。
追加条件`mppi_age`=C_ageは同じ設定で`use_velocity=true, extrapolate_future=false`とし、全ステップの時刻をageに固定する（既定trueは従来どおり）。
`compare_methods.py --comparisons C1-C_age C_age-C2`で比較を選択する。差はC_age−C1、C2−C_age。省略時のC0/C1/C2比較は変わらない。
3条件は同じ`mppi.yaml`、costmap、外形、速度上限、smoother、既存criticを使う。
開発seed=1でC0のclear/staticを確認し、基準MPPIのパラメータは変更していない。

7場面は従来5場面と`stop_go`（進路付近で2秒停止→再発進）、`speed_change`（対向途中で減速）。
`scenarios.yaml`の`segments: [{speed: ..., duration: ...}, ...]`で区間速度を指定する。
旧`speed/duration`形式は1区間として扱う。時刻は目標受付からのシミュレーション秒、区間は左閉右開。
障害物座標は現在の実装ではworld/map座標（基本通路はロボット初期姿勢が原点）。
`_variation`で開始遅延の加算範囲と速度倍率の範囲を指定し、seedから同じ条件を各methodに生成する。
開発seedは1–999、最終評価seedは10000–19999。`repeats`はseedを1ずつ増やす。
範囲外は起動前にエラーにする。最終評価は`seed_set:=final seed:=10000`で明示的に選ぶ。
実際の開始遅延、速度倍率、全区間の速度・時間、seedを`results.csv`に記録する。
seedは障害物条件用。MPPI内部乱数・AMCL・非同期処理のビット一致は保証しない。

追跡器の入力は`/sim/scan`、scan時刻のTF、`/map`（launchで`/sim/map`へremap）。
静的占有セルから0.20m以内の点を除き、隣接点クラスタ・円弧の中心／半径推定・最近傍対応・等速Kalman filterを使う。
真値・シナリオ予定を入力しない。出力は`/sim/obstacle_tracks`、PointCloud2、frame=`map`、
元scan stamp、FLOAT32 fields=`x,y,vx,vy,radius`。
研究条件は`additional_delay:=0.0`（処理後に加えるsim秒）、`velocity_noise_std:=0.0`
（出力速度の各軸へ加える独立Gaussian noiseの標準偏差、m/s）の2つ。
雑音の乱数列は固定seed=0。追跡器の内部ゲートやフィルタ定数は研究条件として露出させない。

criticは追跡位置・速度をlocal costmap frame（`sim/odom`）へ変換し、
`観測から現在の経過秒 + (j+1)*model_dt`で各サンプル位置と比較する。
costmapのパディング済凸footprintを各時刻のyawで回転し、推定障害物円との符号付き距離を求める。
余裕`safety_margin`未満の不足量を全時刻で平均し、`(weight * mean)^power`をコストへ加える。
余裕を含めない距離が1時刻でも負なら、軌道へ一度だけ`collision_cost`（既定1000）も加える。
C1は速度を0にし、C2とC_ageは推定速度を使う。現在フレーム変換に失敗した場合はMPPIへ失敗を通知する。
古い追跡点群（追加遅延+1秒を超えるもの）は使わない。
2秒間隔の`Track prediction`ログに、変換先frame・age・現在／horizon位置を記録する。

採点専用`tracking_errors.csv`は元観測時刻で真値を補間し、最も近い追跡と1m以内で対応付ける。
位置／速度誤差、追跡数、余分な追跡数、未対応を残す。真値速度は障害物odomをworld軸へ回転して照合する。
遮蔽や誤検出を含むため、未対応を捨てて平均誤差だけを比較しない。
`samples.csv`には予定位置、予定速度、位置誤差も残す。

全区間の運動確認（走行比較とは別、ロボットは停止）：

```bash
ros2 launch oit_deliverybot_sim experiment.launch.py \
  scenario:=all method:=mppi seed:=1 motion_check:=true
ros2 run oit_deliverybot_sim check_obstacle_motion.py /path/to/batch
```

`motion_checks.csv`へ数値判定を保存する。位置誤差の許容は0.12m、区間内速度誤差は0.02m/s。
速度の判定は指令切替の前後0.15秒を除外し、全区間の終端+0.5秒まで記録があることも要求する。
位置許容は50ms指令周期・5ms物理ステップの複数切替による離散化を含む。

footprintはcostmapと共通のパディングを含む。等速フィルタは停止・再発進・速度変化に遅れ、AMCL誤差・壁付近の点・
scanの分割から誤追跡が生じ得る。単一円柱の短い確認を人物追跡・多人数・実機性能の証明にはしない。
公開参考は[SLIP](https://github.com/SaranSundar08/SLIP/blob/main/nav2_tgmppi_controller/include/nav2_tgmppi_controller/tools/dynamic_obstacle_cost.hpp)と
[QFN](https://github.com/Arthur1117/QFN-Go1-Dynamic-Navigation/blob/main/src/qfn_mppi_dynamic_critic/src/dynamic_risk_critic.cpp)。
本実装の追跡器・criticはこの入出力と比較に必要な範囲を独自に記述し、コードの転用はしていない。
