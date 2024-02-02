FROM moveit/moveit2:humble-source

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends pip ros-humble-rqt* \ 
   ### You should add those dependencies via the package.xml of your ros package locatet in src/your-ros-package/package.xml
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/* \
   && pip install pyside2 
ENV DEBIAN_FRONTEND=dialog

# You need to add a user "ros" if not using the standart docker image
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DOMAIN_ID=1

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



# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
