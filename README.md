# oit_deliverybot_sim

ROS 2 Jazzy / Gazebo Harmonic の走行実験パッケージ。
仮想LiDAR、AMCL、Nav2、障害物追跡器、MPPI critic、試行の記録・採点をまとめる。
実機ドライバは起動しない。依存は `package.xml` のAPTパッケージを使用する。
地図のオフライン処理のみOpen3D / Pillow / PyYAMLが別途必要。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/ros2_ws
colcon build --packages-select oit_deliverybot_sim --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch oit_deliverybot_sim experiment.launch.py \
  scenario:=clear,crossing method:=mppi,mppi_pos,mppi_cv \
  seed:=1 seed_set:=development gui:=false
```

`mppi` はC0（標準MPPI）、`mppi_pos` はC1（追跡位置を静止物として評価）、
`mppi_cv` はC2（同じcriticで等速予測）。
`scenario:=all` で7場面、`repeats` でseedを増やした反復、`gui:=true` でGazeboを表示する。
初期位置・目標はenvironment YAMLで手動指定する。ROS_DOMAIN_IDは既定71。

結果は `~/ros2_ws/experiments/simulation/batch_…/` にMCAP、試行別CSV・ログ、
全試行の `results.csv` と `comparison.png` を保存する。
失敗・衝突・時間切れ・基盤異常も結果として残す。
開発seedは1–999、最終評価seedは10000–19999で区別する。

設定・学校環境・追跡器とcriticの入出力は [実験の説明](config/simulation/README.md) を参照。
