#!/bin/bash
# Bash script to run a ROS container
# If there is already one running container with the same image it creates a new one along the original.
# If there is no running container it destroys every container using the same image as the one in arguments.
# Containers are stopped when quitted.
#
# Note that to properly work this file need to be put AND execute at the root of the Pi4 project folder (i.e. raspberry_pi_4/) on your host machine.
# 
# Usage Example:
# ./run_waldo_docker.sh


#Define Docker volumes, environment variables and devices
#volume 1 -> used to share X server (graphic) between host and container
#volume 2 -> used to authorize container to use host X server
#volume 3 -> mount host Ros working directory on container Ros workspace
#volume 4 -> mount access to host webcam
DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority" \
--volume="${PWD}":"/Waldo_eurobot_2024":rw \
--volume="/dev/video0:/dev/video0"
"

#env 1 -> specify X server to container (used by gui apps in the container)
#env 2 -> for qt desact inter-processus shared memory while there are graphical app running
DOCKER_ENV_VARS="
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
"

#device 1 -> to connect an usb serial port, used for the lidar
DOCKER_DEVICES="
--device=/dev/ttyUSB0
"

# Define final docker run args
DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}" "${DOCKER_DEVICES}

# Authorize all graphical connection made by a docker container to my pc
xhost + local:docker 1>/dev/null &

wait

#-------------------- Run -----------------------

# Get image name from cli
#image_name="$1"
image_name="waldo_with_ros:latest"

# Get number of same image running
nb_of_running_containers_same_image=$(docker ps -a --filter ancestor="$image_name" --filter status='running' | wc -l)
# Subtract 1 because the title line is counted in 'wc -l'
nb_of_running_containers_same_image=$((nb_of_running_containers_same_image - 1))

if [ "$nb_of_running_containers_same_image" -eq 0 ]; then
    # No running containers -> delete all previous ones with same image
    echo "Suppression des anciens conteneurs de l'image [$image_name]"
    docker stop $(docker ps -a | grep "$image_name" | awk '{print $1}' | xargs -I {} docker stop {}) 1>/dev/null
    docker rm $(docker ps -a | grep "$image_name" | awk '{print $1}' | xargs -I {} docker stop {}) 1>/dev/null
fi

# Dislay info
echo -e "Création d'un nouveau conteneur\nConteneur(s) actif(s) : $(($nb_of_running_containers_same_image+1))"
echo -e "\n----------------------------------------------------\n"
echo -e "Bonjour, bienvenue dans l'environnement de programmation ROS Noetic.\n"
echo -e "Vous serez en mode root, chaque fichier ou répertoire créé dans ce conteneur ne sera modifiable que par vous à l'interieur de ce conteneur.\n"
echo -e "Si vous voulez changer cela utilisez la commande : \n\n\tchmod 777 <nom du fichier ou répertoire>\n"
echo -e "Ainsi, la machine hôte pourra modifier/executer le fichier/répertoire.\n"

# Run
docker run -it --net=host --ipc=host --privileged ${DOCKER_ARGS} "$image_name" bash





