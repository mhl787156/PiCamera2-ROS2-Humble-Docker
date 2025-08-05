FROM ros:humble

RUN apt update && apt install -y --no-install-recommends gnupg

RUN apt update && apt -y upgrade

RUN apt update && apt install -y --no-install-recommends \
        meson \
		ninja-build \
		pkg-config \
		libyaml-dev \
		python3-yaml \
		python3-ply \
		python3-jinja2 \
		libevent-dev \
		libdrm-dev \
		libcap-dev \
		python3-pip \
		python3-opencv \
	&& apt-get clean \
	&& apt-get autoremove \
	&& rm -rf /var/cache/apt/archives/* \
	&& rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install libcamera from source
RUN git clone https://github.com/raspberrypi/libcamera.git && cd libcamera && git checkout 6ddd79b && cd ..
RUN meson setup libcamera/build libcamera/
RUN ninja -C libcamera/build/ install

# Install camera_ros
# RUN mkdir -p /camera_ws/src && cd /camera_ws \
# 	&& git clone https://github.com/christianrauch/camera_ros.git
# # resolve binary dependencies and build workspace
# RUN . /opt/ros/$ROS_DISTRO/setup.sh \
# 	&& cd /camera_ws/ \
# 	&& rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera \
# 	&& colcon build --event-handlers=console_direct+
RUN apt update && apt install -y --no-install-recommends \
		python3-pip git python3-jinja2 \
		libboost-dev libgnutls28-dev openssl libtiff-dev pybind11-dev \
		qtbase5-dev libqt5core5a libqt5widgets5 meson cmake \
		python3-yaml python3-ply \
        ros-humble-camera-ros \
	&& rm -rf /var/lib/apt/lists/*

# Install kmsxx from source
RUN git clone https://github.com/tomba/kmsxx.git
RUN meson setup kmsxx/build kmsxx/
RUN ninja -C kmsxx/build/ install 

# Add the new installations to the python path so that picamera2 can find them
ENV PYTHONPATH=${PYTHONPATH}/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages:/app/kmsxx/build/py

# Finally install picamera2 using pip
RUN pip3 install picamera2

# Setting HOST_UID and running under ros user (mirroring host user id) to enable ros communication with host machine
ARG HOST_UID=1000 
RUN adduser --disabled-password --gecos '' --uid $HOST_UID ros
USER ros

# Copy the test script to the container
COPY camera_test /app/camera_test
COPY run.sh /run.sh

# Set the entry point. You can comment this out to use your own test scripts...
CMD ["./run.sh"]