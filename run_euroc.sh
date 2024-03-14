if pgrep -x "roscore" > /dev/null
then
    echo "roscore is running."
else
    echo "roscore is not running."
    roscore &
fi

# sleep 3 
# rosrun ov2slam ov2slam_node /home/cxn/leofile/ov2slam_ws/src/ov2slam/parameters_files/average/euroc/euroc_mono.yaml &
# sleep 1 
# rosrun rviz rviz -d /home/cxn/leofile/ov2slam_ws/src/ov2slam/ov2slam_visualization.rviz &
# sleep 1 
# rosbag play /home/cxn/leofile/sad_dataset/EuRoc/MH_04_difficult.bag

sleep 3 
# rosrun ov2slam ov2slam_node /home/cxn/leofile/ov2slam_ws/src/ov2slam/parameters_files/accurate/euroc/plusai_mono.yaml &
rosrun ov2slam ov2slam_node /home/cxn/leofile/ov2slam_ws/src/ov2slam/parameters_files/accurate/euroc/plusai_stereo.yaml &
sleep 1 
rosrun rviz rviz -d /home/cxn/leofile/ov2slam_ws/src/ov2slam/ov2slam_visualization.rviz &
sleep 1 
# rosbag play /home/cxn/vo_data/20240220T112329_pdb-l4e-b0001_0_0to59/ros.bag
rosbag play /home/cxn/vo_data/20240313T080709_pdb-l4e-c0001_24_1314to1344/ros.bag