# reachability_map_visualizer

![Screenshot from 2024-12-12 10-23-17](https://github.com/user-attachments/assets/14e43ef1-2e8a-4624-92ff-d6a9a6c99127)

# 環境構築
### choreonoid2の依存パッケージ(必要に応じて)
```
sudo apt install libqt5svg5-dev
```

# ビルドコマンド
```
catkin build reachability_map_visualizer_sample
```
- reachability map本体もこれでビルド可能

# 実行コマンド
### マップ確認
```
choreonoid src/reachability_map_visualizer/reachability_map_visualizer_sample/config/visualize.cnoid
```
### マップ生成
```

```
### パラメタ確認
- 計算領域・EE位置等の確認が可能
- 黄色マーカー：計算領域の中心
- 紫マーカー：重心
- 緑の領域：支持多角形
```
catkin build reachability_map_visualizer_sample --no-deps
source ~/catkin_ws/reachability_map_ws/devel/setup.bash
choreonoid reachability_map_visualizer_sample/config/param_preview.cnoid 
```
### solvability barの表示
```
choreonoid reachability_map_visualizer_sample/config/solvability_bar.cnoid 
```
- figで使いたいときなど
