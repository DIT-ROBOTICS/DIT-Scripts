FROM osrf/ros:humble-desktop

LABEL org.opencontainers.image.authors="scx@gapp.nthu.edu.tw"
LABEL org.opencontainers.image.licenses="MIT"

ENV TZ=Asia/Taipei

RUN apt-get update && apt-get install -y \
    python3-pip \
    portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip install speechrecognition pyaudio

WORKDIR /voice_control_ws
COPY voice_control_ws/ /voice_control_ws/

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /voice_control_ws/install/setup.bash && ros2 run voice_nav speech_to_nav"]
# CMD ["/bin/bash"]
