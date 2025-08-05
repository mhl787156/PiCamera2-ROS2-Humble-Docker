FROM ros:humble

RUN apt update && apt install -y --no-install-recommends gnupg

RUN apt update && apt -y upgrade

RUN apt update && apt install -y --no-install-recommends \
        # meson \
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

RUN pip3 install meson>=0.63
# Install libcamera from source: https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup 
# RUN git clone https://github.com/raspberrypi/libcamera.git && cd libcamera && git checkout 6ddd79b && cd ..
# RUN meson setup libcamera/build libcamera/
# RUN ninja -C libcamera/build/ install

# Install camera_ros
RUN apt update && apt install -y --no-install-recommends \
		python3-pip git python3-jinja2 \
		libboost-dev libgnutls28-dev openssl libtiff-dev pybind11-dev \
		qtbase5-dev libqt5core5a libqt5widgets5 meson cmake \
		python3-yaml python3-ply libglib2.0-dev libgstreamer-plugins-base1.0-dev\
        ros-humble-camera-ros \
	&& rm -rf /var/lib/apt/lists/*


RUN git clone https://github.com/raspberrypi/libcamera.git -b v0.5.1
RUN meson setup libcamera/build libcamera/ --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
RUN ninja -C libcamera/build/ install && ldconfig

# Install kmsxx from source
RUN git clone https://github.com/tomba/kmsxx.git
RUN meson setup kmsxx/build kmsxx/
RUN ninja -C kmsxx/build/ install && ldconfig

# Add the new installations to the python path so that picamera2 can find them
ENV PYTHONPATH=${PYTHONPATH}/usr/local/lib/aarch64-linux-gnu/python3.10/site-packages:/app/kmsxx/build/py

# Finally install picamera2 using pip
# RUN pip3 install picamera2

# Setting HOST_UID and running under ros user (mirroring host user id) to enable ros communication with host machine
ARG HOST_UID=1000 
RUN adduser --disabled-password --gecos '' --uid $HOST_UID ros
USER ros

# Copy the test script to the container
COPY camera_test /app/camera_test
COPY run.sh /run.sh

# Set the entry point. You can comment this out to use your own test scripts...
CMD ["/run.sh"]