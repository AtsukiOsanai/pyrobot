### turtlebot2_calibration
- turtlebot2_control/launch/main.launchでuse_simじゃないときに呼ばれる
- 一応残しておいたが，どことどこの間のキャリブレーションなのかを見極める必要がある。
- おいおいはLidar2camra?

### turtlebot2_control
- armとpan, tiltがないので，srcとincludeは全消し
- teleopのmove_base機能だけ残した(つもり

### turtlebot2_description
- locobotbaseのときからだが，config/joint_names_locobot_description.yamlが必要なのか分からない．  
とりあえず消してみる．
- turtlebot, kobuki_descriptionからパクりまくる．joint, linkが多い時はxacroの方が便利そう
- urdfではなくxacroをgazebo_ros/spawn_modelでspawnさせるときは，  
  <param name="robot_description" command="$(find xacro)/xacro '$(find turtlebot2_description)/robots/kobuki_hexagons_kinect.urdf.xacro'"/>  
としてspawnの引数の-paramにrobot_descriptionを指定する
- urdfが未対応

### turtlebot2_gazebo
- yamlをloadしてrosparamにする機能がある  
launchの中でrosparamでyamlファイルをloadすると取得できる模様

### turtlebot2_navigation
- turtlebot_move_base.launchの中身はturtlebot_navigation packageという外部packageを使っているのでいたずらに変更しないこと。
base_navigationはオリジナルのpkgを指すので名前を変更しても良い。
- turtlebot_main.launchのturtlebot_bringupも外部package


- turtlebot_hogeはどこかと名前がぶつかりそう
turtlebot2にする
