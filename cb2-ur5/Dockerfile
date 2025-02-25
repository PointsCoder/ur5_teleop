# Use Ubuntu 20.04 as base image
FROM ubuntu:20.04

# Set the environment to be non-interactive (for any prompt during the installation)
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH

# Update the package list and install prerequisites
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    software-properties-common \
    build-essential \
    curl \
    wget \
    git \
    ca-certificates \
	lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add deadsnakes PPA (for Python 3.9)
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.9 python3.9-distutils python3.9-venv python3.9-dev && \
    rm -rf /var/lib/apt/lists/*

# Install pip for Python 3.9
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3.9 get-pip.py && \
    rm get-pip.py

# Set python3.9 and pip as default
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1 && \
    update-alternatives --install /usr/bin/pip3 pip3 /usr/local/bin/pip3 1

# Verify Python and pip installation
RUN python3 --version && pip3 --version

# Setting up the source keys for ROS
RUN apt update && apt upgrade -y &&\
	sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
	apt install -y curl && \
	apt install -y gnupg2 gnupg-agent lsb-release && \
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
	apt update && apt install -y ros-noetic-desktop-full && \
	apt-get update && \
	apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    rm -rf /var/lib/apt/lists/* && \
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash"

# Fixing the issue with the netifaces package
RUN python3 -m pip install --target=/opt/ros/noetic/lib/python3/dist-packages netifaces

# Initialize rosdep
RUN rosdep init && rosdep update

# Install and use catkin
RUN apt-get update && \
	apt-get install -y python3-catkin-tools ros-noetic-catkin python3-osrf-pycommon && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash" && export PATH=$PATH:/opt/ros/noetic/bin && \
	mkdir -p ~/catkin_ws/src && \
	cd ~/catkin_ws && \  
	/bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3.9 -DPYTHON_INCLUDE_DIR=/usr/include/python3.9 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.9.so" && \
	echo "source devel/setup.bash" >> ~/.bashrc && \
	/bin/bash -c "source devel/setup.bash"

# Setting up libfcl correctly
RUN sudo apt-get remove --purge libfcl-dev -y && \
	sudo apt-get install -y cmake libeigen3-dev && \
	mkdir ~/packages && cd ~/packages && \
	git clone -b 0.6.1 https://github.com/flexible-collision-library/fcl.git && \
	cd fcl && \
	mkdir build && cd build && \
	cmake .. -DCMAKE_BUILD_TYPE=Release && \
	make -j$(nproc) && \
	sudo make install && \
	ls /usr/local/lib | grep libfcl && \
	echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/fcl.conf && \
	sudo ldconfig
	
# Install all the dependencies necessary for Moveit
RUN export PATH=$PATH:/opt/ros/noetic/bin && \
	python3 -m pip install numpy && \
	sudo apt-get install -y ros-$ROS_DISTRO-universal-robots && \
	cd ~/catkin_ws/src && \
	git clone https://github.com/ros-industrial/ur_modern_driver.git && \
	cd ur_modern_driver/ && \
	git checkout kinetic-devel && \
	cd ../ && \
	git clone https://github.com/ros-controls/ros_control.git && \
	git clone https://github.com/ros-industrial/industrial_core.git && \
	sudo apt install -y ros-noetic-catkin python3-catkin-tools && \
	sudo apt install -y python3-wstool && \
	git clone https://github.com/ros-planning/moveit_tutorials.git -b master && \
	git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel && \
	git clone https://github.com/ros-planning/moveit.git && \
	cd moveit/ && \
	git checkout noetic-devel && \
	cd ../ && \
	git clone https://github.com/ros-industrial/universal_robot.git && \
	cd universal_robot/ && \
	git checkout kinetic-devel && \
	cd ../ && \
	rosdep install -y --from-paths . --ignore-src --rosdistro noetic && \
	pip install setuptools==58.0.4 && \
	pip install testresources && \
	cd ../ && \
	python3 -m pip install --target=/opt/ros/noetic/lib/python3/dist-packages numpy && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash && catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release" && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make" && \
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Giving GUI access to Docker
RUN export DISPLAY=:1 && sudo apt-get install -y mesa-utils

# Set the working directory in the container
WORKDIR /app

# Copy the current directory contents into the container
COPY . /app