## Locobot control
- includeとsrcの内容はarm, gripper, pan, tiltの動作に関するもの
    - armとgripperに関する部分を順次消去
    - srvはarm, gripperについての動作なので消しても良い
    - joint_statesってカメラのpan, tiltも含まれている？  
    含まれている．sensor_msgs::joint_stateはvectorに動かしたい部位を順次push_backで登録する．  
    その中にpan, tiltも含まれている
    - torque_controlはpan, tiltにも含まれている？
    含まれていない
    - execute_joint_trajectory
      - armにしか関係していなそう
      - initActionlibで呼ばれている
      - smooth_joint_trajectory_server_がセット＆スタートされる
    - control_msgs::FollowJointTrajectoryActionが関連，不要になる？
    - NodeHandle::createTimerで周期を設定したcallbackを作れる
    - DynamixelWorkBench dxl_wb_はpan, tileでも使用．消せない
    - initDynamixelsはいる？
    おそらくpan, tiltのためにも必要．arm, gripperの部分は消去する.
    - arm, gripper, pan, tiltが不要ならdynamixel関連の部分は不要
    - dyamixel_mutex_, hardware_mutex_は必要？
    - dynamixel_name_2idsはgetDynamixelsInfoで取得
    - control_items_は必要？
    dynamixel motorごとの情報を取得する部分に見えるので，pan, tiltを保持するなら必要そう
    - pan, tiltも消す場合は，src, includeを全消去
- scripts/active_camera.pyもいま時点では消す  
人追従の機能とを作りたくなったらココを編集すればできる？


## locobot_calibration
- armを動かしながらar_track_alvar_calibrationというpackageを使ってcalibrationを行う
- ここにlidar-camera calibrationが入れられると良い

## locobot_navigation
#### base_navigation
- config/createは消した. createはlocobot_liteのためのもののよう
- 全体のlaunchはlocobot_control/launch/main.launchより
  - 実機の時は，navigation/launch/main_base.launch --> turtlebot_main.launch --> diagnostic_aggregator/aggregator_nodeを起動
  - sim, 実機ともにmove_base.launchを起動
  - baseがkobukiの場合はturtlebot_move_base.launchがmove_base.launchの中で呼ばれる
#### orb_slam2_navigation
- cfgにはcamera parameterを置く
- orb_slam2_realsense.launchはlocobot_control/launch/main.launchから呼ばれている．実機動作時のみ
simでは使えないのか？ステレオ点群でgmappingしとけってこと？

## locobot_description
- urdfはjointとlinkからなる
  - link
  物体の情報(形状やmeshなど)
    - visual  
    STLやdaeで外観を反映
    - collision  
    接触判定. primitiveでモデル化したりする
    - inertial
  - joint
    - child and parenet frame, frame間の位置関係を記述
    - parent link
    - child link
    - origin: parent linkを基準とした原点のオフセット, 回転
    - axis: 回転体のときは回転軸を記述
    - 回転するjointの場合はtype='revolute'
    limit tagもセットで記述が必要
  - transmission
  動くものを定義する場合に必要そう  
  transmission_interface, hardware_interfaceの記述が必要
  - cameraもここで定義  
  plugin(libgazebo_ros_openni_kinect.so)を使用  
  camera_parametersが初期値っぽいのでどこか別の場所で反映されている？
- wheelの回転ってz軸なの？--> rpy=(-pi/2, 0, 0)が指定されているため
- camera_linkはどこにある？レンズ中心か付け根か．rvizで要確認
- libgazebo_ros_controlはお決まりで入れるみたい
- kobukiのdrive部分が見つからないがどこだ？
- STLのincludeはlocobot_descriptionからloadできるかも  
二重にファイルを持つ必要はなさそう
- display.launchでrobotの形状を見るだけができる

## locoot_gazebo
- gazeboで動かすserviceなどを定義する場合はここを編集

## Others
- 別のthird party packageを使う場合
  - .gitmodulesにurlとpathを追加
  - pathはpackageを使うロボットに紐付ける

