import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/svasude7/HRA/RL_Active_Multi_Target_Tracking/model_based_active_mapping/ros2_ws/install/mbam_gazebo_tracking'
