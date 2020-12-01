gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_gazebo sitl_mocap_formation.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_experiment prometheus_mocap_formation_control.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_experiment prometheus_mocap_formation_setmode.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch prometheus_experiment prometheus_mocap_formation_change.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun prometheus_mission mocap_formation_move; exec bash"' \
