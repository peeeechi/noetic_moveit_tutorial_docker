ARG ROS_DISTRO=noetic
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG USER_NAME=inoue
ARG GROUP_NAME=inoue
ARG _UID=1000
ARG _GID=1000
ARG ROS_DISTRO=noetic
ARG DEBIAN_FRONTEND=noninteractive
ARG PASSWORD=${USER_NAME}
ENV USER_NAME=${USER_NAME}
ENV GROUP_NAME=${GROUP_NAME}
ENV UID=${_UID}
ENV GID=${_GID}
ENV LANG='ja_JP.UTF-8'
ENV TZ='Asia/Tokyo'

# iproute2 \
# openssh-server \
# netcat-openbsd \
# python3-rosdep \

RUN apt-get update && apt-get install -y --no-install-recommends \
    gosu \
    ca-certificates \
    apt-transport-https \
    vim \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# 日本語設定
    # language-pack-ja-base：日本語の翻訳パック
    # language-pack-ja：日本語の翻訳パック（更新版）
    # fonts-ipafont-gothic: IPA ゴシック フォント
RUN apt-get update && apt-get install -y --no-install-recommends \
        language-pack-ja-base \
        language-pack-ja \
        fonts-ipafont-gothic \
    && rm -rf /var/lib/apt/lists/*/apt/lists/*

# Python3をpythonにリンク
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.8 2

# Automake 関係のインストール
# RUN apt-get update && apt install -y --no-install-recommends \
#         automake \
#         autoconf \
#         libtool \
#     && rm -rf /var/lib/apt/lists/*

# pip のインストール
RUN curl -L https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python get-pip.py; \
    rm -f get-pip.py

# ROS 関連ツールのインストール

# ros-${ROS_DISTRO}-industrial-robot-status-interface \
# ros-${ROS_DISTRO}-industrial-robot-status-controller \
# ros-${ROS_DISTRO}-ur-client-library \
# ros-${ROS_DISTRO}-code-coverage \
# ros-${ROS_DISTRO}-rosdoc-lite
# ros-${ROS_DISTRO}-pybind11-catkin \


    # ros-${ROS_DISTRO}-ps3joy \
    # ros-${ROS_DISTRO}-wiimote \
    # ros-${ROS_DISTRO}-joystick-drivers \
RUN apt-get update \
    && apt-get -y install \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-jsk-recognition \
    ros-${ROS_DISTRO}-jsk-rviz-plugins \
    ros-${ROS_DISTRO}-plotjuggler \
    ros-${ROS_DISTRO}-code-coverage \
    ros-${ROS_DISTRO}-joy \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

#   python3-tk \
#   mysql-client \
#   gdb \
RUN apt-get update \
  && apt-get dist-upgrade -y \
  && apt-get -y install \
  python3-rosinstall \
  python3-catkin-tools \
  python3-osrf-pycommon \
  python3-wstool \
  fonts-freefont-ttf \
  && rm -rf /var/lib/apt/lists/*
  # python3-wstool \


# 関連ツールのインストール
# ccache: C,C++のビルド結果のキャッシュ
# xvfb: Xvfb (X virtual framebuffer) は X Window System の仮想ディスプレイを作ることのできるソフトウェア
# RUN apt-get update && apt install -y --no-install-recommends \
#         xvfb \
#         ccache \
#         policykit-1 \
#         rsync \
#         sshpass \
#     && rm -rf /var/lib/apt/lists/*


# 日本語入力モジュールのインストール
# 参考: https://zenn.dev/ysuito/articles/c196ae84c95e31
# RUN apt update && apt install -y --no-install-recommends \
#         dbus-x11 \
#     && rm -rf /var/lib/apt/lists/*

# RUN apt update && apt install -y --no-install-recommends \
#         python3-venv \
#     && rm -rf /var/lib/apt/lists/*

ARG ROOT_WS=/root/catkin_ws

RUN rosdep update \
  && apt-get update \
  && apt-get dist-upgrade -y \
  && mkdir -p ${ROOT_WS}/src

WORKDIR ${ROOT_WS}/src

COPY ./dotfiles/.bashrc /tmp/dotfiles/.bashrc

RUN wstool init . \
  && wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall \
  && wstool remove  moveit_tutorials \
  && wstool update -t . \
  && git clone https://github.com/ros-planning/moveit_tutorials.git -b master \
  && rosdep install -y --from-paths . --ignore-src --rosdistro noetic \
  && echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc


WORKDIR ${ROOT_WS}


# RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   && catkin build

# rust install
RUN curl --proto '=https' --tlsv1.2 -sSf -o /tmp/rustup.sh https://sh.rustup.rs \
  && chmod 777 /tmp/rustup.sh
#   && chown ${USER_NAME}:${GROUP_NAME} /tmp/rustup.sh \
  # && /tmp/rustup.sh -y

RUN /tmp/rustup.sh -y
WORKDIR /root

RUN . ~/.bashrc && rustup target add x86_64-pc-windows-gnu

COPY ./scripts/init.sh /tmp/init.sh
COPY ./dotfiles/.vimrc /tmp/dotfiles/.vimrc
RUN chmod 775 /tmp/init.sh

ENTRYPOINT [ "/tmp/init.sh" ]
CMD [ "/bin/bash" ]

