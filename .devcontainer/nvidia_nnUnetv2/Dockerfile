FROM nvidia/cuda:12.2.0-base-ubuntu22.04

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive

#ROS Installation
 
RUN locale  
RUN  apt update &&  apt -y install --no-install-recommends locales
RUN  locale-gen en_US en_US.UTF-8
RUN  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
 
RUN locale  # verify settings

RUN  apt -y install --no-install-recommends software-properties-common
RUN  add-apt-repository universe

RUN  apt update &&  apt -y install --no-install-recommends curl -y
RUN  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN  apt update &&  apt -y install --no-install-recommends ros-dev-tools

RUN  apt update
RUN  apt update
RUN  apt -y install --no-install-recommends ros-iron-desktop


#ROS Installation done

RUN apt-get update \
   && apt-get -y install --no-install-recommends openssh-client libxcb-xinerama0 libxcb-xkb1 pip\
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*


RUN pip3 install torch torchvision torchaudio

RUN pip install nnunetv2

ENV DEBIAN_FRONTEND=dialog
# RUN pip install pre-commit
# Set up auto-source of workspace for ros user
ARG USERNAME=ros

ARG USER_UID=1000

ARG USER_GID=$USER_UID

ARG DOMAIN_ID=129

# Create a non-root user

RUN groupadd --gid $USER_GID $USERNAME \
 && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#  [Optional] Add sudo support for the non-root user
 && apt-get update \
 && apt-get install -y sudo \
 && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
 && chmod 0440 /etc/sudoers.d/$USERNAME \
 # Cleanup
 && rm -rf /var/lib/apt/lists/* \
 && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
 && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

RUN echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

