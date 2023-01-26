#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROSスピニングは、ロボットの状態に関する情報を取得するために、MoveGroupInterfaceのために実行されている必要があります。
  // これを行う1つの方法は、事前にAsyncspinnerを開始することです。
  
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  // Setup
  // MoveITは、「計画グループ」と呼ばれるジョイントのセットを操作し、「JointModelGroup」と呼ばれるオブジェクトに保存します。
  // MoveIT全体で「計画グループ」と「共同モデルグループ」という用語が同じ意味で使用されます.

  static const std::string PLANNING_GROUP = "manipulator";
  // static const std::string PLANNING_GROUP = "endeffector";

  // planning_interface::MoveGroupInterfaceクラスは、制御および計画したい計画グループの名前のみを使用して簡単にセットアップできます。
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  
  // planning_interface::planningsceneinterface クラスを使用して、「仮想世界」シーンで衝突オブジェクトを追加および削除します
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  // 生のポインターは、パフォーマンスの向上のために計画グループを参照するために頻繁に使用されます。
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node_handle);
  // moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  /**
   * Visualization
   * MoveITVisualToolsは、RVIZのオブジェクト、ロボット、軌跡を視覚化するための多くの機能を提供し、
   * スクリプトのステップバイステップの内省などのデバッグツールを提供します。
   */
  namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("/endeffector");
  // moveit_visual_tools::MoveItVisualTools visual_tools("/manipulator");
  // moveit_visual_tools::MoveItVisualTools visual_tools("tool0", rvt::RVIZ_MARKER_TOPIC, moveit_cpp_ptr->getPlanningSceneMonitor());
  moveit_visual_tools::MoveItVisualTools visual_tools("world", rvt::RVIZ_MARKER_TOPIC);
  // moveit_visual_tools::MoveItVisualTools visual_tools("world");
  // moveit_visual_tools::MoveItVisualTools visual_tools("/odom");
  visual_tools.deleteAllMarkers();
  
  // リモートコントロールは、RVIZのボタンとキーボードショートカットを介して高レベルのスクリプトをステップスルーできるようにする内省ツールです。
  visual_tools.loadRemoteControl();
  
  // RVIZはさまざまな種類のマーカーを提供します。
  // このデモでは、テキスト、シリンダー、球体を使用します
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().x() = 1.0;
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  
  // バッチパブリッシングは、大規模な視覚化のためにRVIZに送信されるメッセージの数を減らすために使用されます
  visual_tools.trigger();

  // Getting Basic Information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  auto link_names = move_group_interface.getLinkNames();
  for (size_t i = 0; i < link_names.size(); i++)
  {
    ROS_INFO_NAMED("tutorial", "Link name: %s", link_names[i].c_str());
  }

  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(), move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  
  // Start DEMO
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to start the demo");
  
  // move_group_interface planning-to-pose-goal:
  // このグループのモーションを、エンドエフェクターの目的のポーズに計画することができます。
  
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 1.0;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.0;
  // target_pose1.orientation.w = 1.0;

  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;

  move_group_interface.setPoseTarget(target_pose1);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
  
  // Visualizing plans

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to continue the demo");
  
  // 最後に、my_planに保存されている軌跡を実行するには、次のメソッドコールを使用できます。
  // ロボットがその間に移動した場合に問題につながる可能性があることに注意してください。
  
  // move_group_interface.execute(my_plan);

  // Moving to a pose goal
  
  // 計画された軌道を検査したくない場合は、以下は上記の2段階のプラン+実行パターンのより堅牢な組み合わせであり、優先される必要があります。
  // 私たちが以前に設定したポーズ目標はまだアクティブであるため、ロボットはその目標に移行しようとすることに注意してください.
  
  // move_group_interface.move();
  
  // Planning to a joint-space goal
  // Jointスペースの目標を設定し、それに向かって進みましょう。これにより、上記のポーズ目標を置き換えます。

  // まず、現在のロボット状態を参照するポインターを作成します。
  // RobotStateは、すべての現在の位置/速度/加速度データを含むオブジェクトです。
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  
  // 次に、Jointグループの現在値のセットを取得します.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  
  // それでは、ジョイントの1つを変更し、新しいジョイントスペースの目標に計画し、計画を視覚化しましょう。
  joint_group_positions[0] = -tau / 6;  // -1/6ラジアンで回転します
  move_group_interface.setJointValueTarget(joint_group_positions);
  
  
  
  // 許容される最大速度と加速度を最大の5％に下げます。
  // デフォルト値は10％（0.1）です。
  // ロボットのmovot_configのgoint_limits.yamlファイルで、ロボットをより速く移動する必要がある場合は、
  // コード内の明示的な要因を設定するyamlファイルでお好みのデフォルトを設定します。
  double max_velocity = 0.05;
  double max_acceleration = 0.05;
  move_group_interface.setMaxVelocityScalingFactor(max_velocity);
  move_group_interface.setMaxAccelerationScalingFactor(max_acceleration);
  
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success? "SUCCESS" : "FAILED");

  // Visualize the plan in RViz

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to continue the demo");
  
  // Planning with Path Constraints
  //
  // パスの制約は、ロボットのリンクに簡単に指定できます。
  // 私たちのグループのパスの制約とポーズ目標を指定しましょう。
  
  
  // 最初にパスの制約を定義します.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "";
  ocm.header.frame_id = "";
  ocm.orientation.w = 1.0;

  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  
  // 次に、グループのパス制約として設定します。
   
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group_interface.setPathConstraints(test_constraints);

  // Enforce Planning in Joint Space
  //
  // 計画の問題に応じて、MoveITは問題の表現のために「joint space」と「cartesian space」を選択します.
  // ompl_planning.yamlファイルでグループパラメーター `` endforce_joint_model_state_space：true``を設定します。
  //
  // デフォルトでは、オリエンテーションパスの制約を備えた計画リクエストは、
  //「デカルトスペース」でサンプリングされます。これにより、IKを呼び出すことは生成サンプラーとして機能します。
  //
  // 「ジョイントスペース」を実施することにより、計画プロセスは拒否サンプリングを使用して有効な要求を見つけます。
  // これにより、計画時間が大幅に増加する可能性があることに注意してください。
  //
  // 私たちは持っていた古い目標を再利用し、それに計画します。
  // これは、現在の状態がすでにパスの制約を満たしている場合にのみ機能することに注意してください。
  // したがって、開始状態を新しいポーズに設定する必要があります。
  
  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  geometry_msgs::Pose start_pose2;
  // start_pose2.orientation.w = 1.0;
  start_pose2.orientation.w = 0.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group_interface.setStartState(start_state);

  // 次に、作成したばかりの新しいスタート状態から、以前のポーズターゲットを計画します。
  move_group_interface.setPoseTarget(target_pose1);
  
  // すべてのサンプルが逆運動ソルバーを呼び出す必要があるため、制約のある計画は遅くなる可能性があります。
  // プランナーが成功するのに十分な時間があることを確認するために、デフォルトの5秒から計画時間を増やすことができます。
  double planning_secs = 10.0;
  move_group_interface.setPlanningTime(planning_secs);
  
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "SUCCESS" : "FAILED");

  // Visualize the plan in RViz
  
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'Next' ");

  // パス制約で完了したら、必ずクリアしてください.
  move_group_interface.clearPathConstraints();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // エンドエフェクターが通過できるウェイポイントのリストを指定することにより、デカルトパスを直接計画できます。 
  // 上記の新しい開始状態から始めていることに注意してください。
  // 最初のポーズ（Start State）はウェイポイントリストに追加する必要はありませんが、視覚化に役立つ可能性があります
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);
  geometry_msgs::Pose target_pose3 = start_pose2;
  
  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3); // down
  
  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3); // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3); // up and left

  // 1 cmの解像度でデカルトパスを補間することを望みます。
  // そのため、0.01をデカルト翻訳の最大ステップとして指定します。
  // ジャンプしきい値を0.0として指定し、効果的に無効にします。
  //
  // 警告 - 実際のハードウェアを操作しながらジャンプしきい値を無効にすると、
  //        冗長ジョイントの予測不可能な動きが大きくなり、安全性の問題になる可能性があります
  
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved", fraction * 100.0);
  
  // Visualize the plan in RViz

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i =0; i < waypoints.size(); ++i) {
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  }
  visual_tools.trigger();
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to continue the demo ");
  
  // デカルトの動きはしばしば遅くなければなりません。例えば、オブジェクトに近づくときなど。
  // 現在、デカルトプランの速度をMaxVelocityScalingFactorを介して設定することはできませんが、
  // ただし、ここ(https://groups.google.com/forum/#!topic/moveit-users/MOoFxy2exT4)で説明するように、手動で軌跡を計算する必要があります 
  // プルリクエストは大歓迎です。

  move_group_interface.execute(trajectory);

  // Adding objects to the environment
  //
  // まず、オブジェクトが邪魔されない別の簡単な目標を計画しましょう。
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.x = 1.0;
  another_pose.position.x = 0.7;
  another_pose.position.y = 0.0;
  another_pose.position.z = 0.59;
  move_group_interface.setPoseTarget(another_pose);
  
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (with no obstacles) %s", success? "SUCCESS" : "FAILED");
  
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Clear Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Next step");

  // 結果は次のようになるかもしれません：
  //
  // .. image:: ./move_group_interface_tutorial_clear_path.gif
  //    :alt: 目標に向かって比較的まっすぐに動いている腕を示すアニメーション
  //


  // 次に、避けるためにロボットの衝突オブジェクトROSメッセージを定義しましょう。
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();

  // オブジェクトのIDは、それを識別するために使用されます。
  collision_object.id = "box1";
  
  // 追加するBoxを定義します
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;
  

  // ボックスのポーズを定義します（frame_idに対して指定）
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5; 
  box_pose.position.y = 0.0; 
  box_pose.position.z = 0.25; 
  
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);


  // それでは、衝突オブジェクトを世界に追加しましょう
  //（追加のオブジェクトを含む可能性のあるベクトルを使用してください）
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  // ステータスのrvizにテキストを表示し、movegroupが衝突オブジェクトメッセージを受信して処理するのを待ちます
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to once the collision object appears in RViz");


  // 今、私たちが軌道を計画するとき、それは障害を回避します
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "SUCCESS" : "FAILED");
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window once the plan is complete"); 
    
  // 結果は次のようになるかもしれません：
  //
  // .. image:: ./move_group_interface_tutorial_avoid_path.gif
  //    :alt: 新しい障害を避けて動いている腕を示すアニメーション
  //
  // Attaching objects to the robot
  //
  // ロボットのジオメトリで移動するように、ロボットにオブジェクトをattachできます.
  // これは、オブジェクトを操作する目的でオブジェクトをピックアップすることをシミュレートします。
  // モーション計画は、2つのオブジェクト間の衝突も回避する必要があります。
  
  moveit_msgs::CollisionObject object_to_attach;
  object_to_attach.id = "cylinder1";

  shape_msgs::SolidPrimitive cylinder_primitive;
  // cylinder_primitive.type = cylinder_primitive.CYLINDER;
  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

  // グリッパーに表示されるように、このシリンダーのフレーム/ポーズを定義します
  object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
  geometry_msgs::Pose grab_pose;
  grab_pose.orientation.w = 1.0;
  grab_pose.position.z = 0.2;

  // まず、オブジェクトを世界に追加します（ベクトルを使用せずに）
  object_to_attach.primitives.push_back(cylinder_primitive);
  object_to_attach.primitive_poses.push_back(grab_pose);
  object_to_attach.operation = object_to_attach.ADD;
  planning_scene_interface.applyCollisionObject(object_to_attach);

  // 次に、オブジェクトをロボットに「attach」します。frame_idを使用して、接続されているロボットリンクを決定します。
  // ApplyAttachedCollisionObjectを使用して、オブジェクトをロボットに直接接続することもできます。
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group_interface.attachObject(object_to_attach.id, "ee_link");
  
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // MoveGroupが添付の衝突オブジェクトメッセージを受信して処理するのを待ちます
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window once the new object is attached to the robot");
  
  // Replan, but now with the object in hand.
  move_group_interface.setStartStateToCurrentState();
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window once the plan is complete");
  
  // 結果は次のように見えるかもしれません：
  //
  // .. image:: ./move_group_interface_tutorial_attached_object.gif
  //    :alt: オブジェクトが添付されたら、腕が異なる動きを示すアニメーション
  //
  // Detaching and Removing Objects
  //
  // それでは、ロボットのグリッパーからシリンダーを取り外しましょう。
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group_interface.detachObject(object_to_attach.id);
  
  // ステータスのrvizでテキストを表示します
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // MoveGroupが添付の衝突オブジェクトメッセージを受信して処理するのを待ちます
  visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window once the new object is detached from the robot");

  // Now, let's remove the objects from the world.
  ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);


  // ステータスのrvizでテキストを表示します
  visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // MoveGroupが添付の衝突オブジェクトメッセージを受信して処理するのを待ちます
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");
  
  ros::shutdown();
  
  return 0;
}