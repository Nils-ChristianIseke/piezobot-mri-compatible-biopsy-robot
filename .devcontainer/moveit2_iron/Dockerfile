FROM moveit/moveit2:iron-source
# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
   && apt-get -y install --no-install-recommends openssh-client libxcb-xinerama0 libxcb-xkb1 ros-${ROS_DISTRO}-rqt* pip gdb libxtst6 \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*



RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
RUN pip3 install vtk pyvista nibabel scikit-spatial
RUN pip install --upgrade pip setuptools
# RUN pip install nnunetv2

# RUN git clone https://github.com/MIC-DKFZ/nnUNet.git && cd nnUNet && pip install -e .

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
