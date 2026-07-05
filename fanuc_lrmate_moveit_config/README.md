

ros2 launch fanuc_lrmate_moveit_config fanuc_moveit_lrmate.launch.py use_mock:=true


ros2 launch fanuc_lrmate_moveit_config fanuc_moveit_lrmate.launch.py \
  robot_model:=lrmate200id7l \
  robot_series:=lrmate \
  moveit_config:=fanuc_lrmate_moveit_config \
  use_mock:=true



ros2 launch fanuc_lrmate_moveit_config fanuc_moveit_lrmate.launch.py \
  robot_model:=lrmate200id7l \
  robot_series:=lrmate \
  moveit_config:=fanuc_lrmate_moveit_config \
  use_mock:=false \
  robot_ip:=192.168.1.100




