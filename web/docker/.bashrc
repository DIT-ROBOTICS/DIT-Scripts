# ==================== ROS ==================== #
# ==== Description: ROS environment setup ===== #
# =================== BEGIN =================== #

source_ros_environment() {
    if [ "$ROS_DISTRO" = "humble" ]; then
        # Custom Alias
        alias rosdep-check='rosdep install -i --from-path src --rosdistro humble -y'
        alias build='colcon build --symlink-install'

        # Source ROS environment
        source /opt/ros/humble/setup.bash
        export ROS_DOMAIN_ID=${HOSTNAME##*-}

        # Source workspace environment
        # latest_setup_bash: Find the latest setup.bash over user's root directory based on the last modified time
        latest_setup_bash=$(find $(pwd) -type f -name "setup.bash" -wholename "*/install/setup.bash" -printf "%T@ %p\n" | sort -nr | awk '{print $2}' | head -n 1)
        if [ -n "$latest_setup_bash" ]; then
            source $latest_setup_bash
            # Source colcon environment
            source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
            source /usr/share/colcon_cd/function/colcon_cd.sh
            export _colcon_cd_root="${latest_setup_bash%/install/setup.bash}"
        else
            echo "No setup.bash file found for humble"
        fi
    elif [ "$ROS_DISTRO" = "noetic" ]; then
        source /opt/ros/noetic/setup.bash
        # Source workspace environment
        if [ -n "$ROS_WS" ]; then
            source $ROS_WS/devel/setup.bash
        else
            echo "ROS_WS variable is not set for noetic"
        fi
    else
        echo "Unsupported ROS_DISTRO: $ROS_DISTRO"
    fi
}

# ==================== END ==================== #



# ================= Functions ================= #
# Note: If you want to use the custom function, #
#        you need to uncomment the line below.  #
# =================== BEGIN =================== #

source_ros_environment

# ==================== END ==================== #
