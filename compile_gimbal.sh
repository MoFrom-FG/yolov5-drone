# compile common message
catkin_make --source Modules/common/msgs --build build/msgs
# compile object_detection
catkin_make --source Modules/object_detection --build build/object_detection
# compile gimbal_control
catkin_make --source Modules/gimbal_control --build build/gimbal_control


