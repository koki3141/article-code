# セットアップの仕方
1. 事前準備

    次のソフトウェアは事前に準備していてください．
    * WSL2
    * Docker
    * Git

2. コンテナの作成

    まず，コンテナの作成をします．
    dockerディレクトリに移動し，次のコマンドを実行してください．
    ```
    docker compose up -d
    ```
3. lbr_fri_ros2_stackをセットアップ

    次に[lbr_fri_ros2_stack](!https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/dev-humble-sequenced-motion)をセットアップをします．
    まず，コンテナ内のターミナルを起動してください．
    次のコマンドを実行し，lbr_fri_ros2_stack_setup.shのコマンドを実効してください．
    lbr_fri_ros2のセットアップとROSのビルドを行います．
    ```
    . .devcontainer/lbr_fri_ros2_stack_setup.sh
    ```

以上で環境構築は終了です．
確認のため，次のコマンドを実行してみましょう


```
source install/setup.bash
ros2 launch lbr_moveit_cpp hello_moveit.launch.py \
    mode:=mock \
    model:=iiwa7 # [iiwa7, iiwa14, med7, med14]
```

Rviz上のロボットが動かされることが確認できます．
以上で確認作業は終わりです．



## 実行
```
ros2 launch lbr_iiwa7_robot_state robot_state_service.launch.py
```

```
 ros2 launch lbr_iiwa7_rrtstar_path_planing robot_state_client.launch.py
 ```
## 参考文献
* https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble
* https://lbr-stack.readthedocs.io/en/latest/index.html#
* https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/lbr_moveit_cpp/doc/lbr_moveit_cpp.html
