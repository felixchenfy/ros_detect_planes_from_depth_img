set -e

# 1. Publish color and depth images.

# Download color/depth images publisher.
ROOT=$(rospack find ros_detect_planes_from_depth_img)
cd ${ROOT}/.. # cd to catkin src/ folder.
if [ ! -d "ros_pub_and_sub_rgbd_and_cloud" ]; then
    repo="https://github.com/felixchenfy/ros_pub_and_sub_rgbd_and_cloud"
    echo "Installing rgbd images publisher: ${repo}"
    git clone ${repo}
    cd ros_pub_and_sub_rgbd_and_cloud
    chmod a+x pub_rgbd_and_cloud.py
fi
cd ${ROOT}

publish_images(){
    ROOT=$(rospack find ros_detect_planes_from_depth_img)
    rosrun ros_pub_and_sub_rgbd_and_cloud pub_rgbd_and_cloud.py \
        --base_dir $ROOT \
        --config_file $ROOT/config/pub_rgbd_config.yaml
}

# 2. Start plane detection server.
run_plane_detection(){
    ROOT=$(rospack find ros_detect_planes_from_depth_img)
    rosrun ros_detect_planes_from_depth_img run_server.py \
        --config_file $ROOT/config/plane_detector_config.yaml \
        --depth_topic test_data/depth \
        --color_topic test_data/color \
        --camera_info $ROOT/data/cam_params_realsense.json
}

# 3. View results in rviz.
view_results_in_rviz(){
    ROOT=$(rospack find ros_detect_planes_from_depth_img)
    rosrun rviz rviz -d $ROOT/config/rviz_for_test_data.rviz
}

# 4. Run all the above scripts in parallel.
# https://stackoverflow.com/questions/3004811/how-do-you-run-multiple-programs-in-parallel-from-a-bash-script
(trap 'kill 0' SIGINT; publish_images & run_plane_detection & view_results_in_rviz) 

