#!/bin/bash -eu


# create user 
groupadd -g ${GID} ${GROUP_NAME}
useradd -u ${UID} -g ${GID} ${GROUP_NAME}
echo "${USER_NAME}:${PASSWORD}" | chpasswd
usermod -G sudo ${GROUP_NAME}
cp -a /tmp/dotfiles/. /home/${USER_NAME}
chown -R ${USER_NAME}:${GROUP_NAME} /home/${USER_NAME}

# move rust env

dirs=(.rustup .cargo .bashrc)

for dir in ${dirs[@]}; do
    echo -e "copy /root/${dir} ...\n"
    cp -r /root/${dir} /home/${USER_NAME} \
        && chmod -R 774 /home/${USER_NAME}/${dir} \
        && chown -R ${USER_NAME}:${GROUP_NAME} /home/${USER_NAME}/${dir}
done

echo "source /home/${USER_NAME}/.cargo/env" >> /home/${USER_NAME}/.bashrc

# move ros env
if [ -z "$(ls /home/${USER_NAME}/catkin_ws)" ]; then
    cp -R /root/catkin_ws/ /home/${USER_NAME}
fi
cp -R /root/.ros/ /home/${USER_NAME}
chown -R ${USER_NAME}:${GROUP_NAME} /home/${USER_NAME} 

echo "USER_NAME: ${USER_NAME}"
echo "GROUP_NAME: ${GROUP_NAME}"

gosu ${USER_NAME}:${GROUP_NAME} "$@"