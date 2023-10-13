#! /bin/sh


# configuring the security

# installing from source
sudo apt update
sudo apt install libssl-dev

colcon build --symlink-install --cmake-args -DSECURITY=ON


# executing the demo
# making a folder for the security files
mkdir ~/sros2_demo

# generating a keystore
cd ~/sros2_demo
ros2 security create_keystore demo_keystore

# generating keys and certificates
ros2 security create_enclave demo_keystore /talker_listener/talker
ros2 security create_enclave demo_keystore /talker_listener/listener


# setting up environment variables
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# running talker and listener demo
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
# in new terminal
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener









## ensuring security across machines

# creating the second keystore
ssh Bob
mkdir ~/sros2_demo
exit

# copy files
cd ~/sros2_demo/demo_keystore
scp -r talker USERNAME@Bob:~/sros2_demo/demo_keystore

# launching nodes
cd ~/sros2_demo/demo_keystore
scp -r talker USERNAME@Bob:~/sros2_demo/demo_keystore
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener








## examining network traffic
# installing tcpdump
sudo apt update
sudo apt install tcpdump

# starting the talker and listener
# Disable ROS Security for both terminals
unset ROS_SECURITY_ENABLE

# In terminal 1:
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

# In terminal 2:
ros2 run demo_nodes_cpp listener --ros-args --enclave /talker_listener/listener



# displaying unencrypted discovery packets
sudo tcpdump -X -i any udp port 7400

sudo tcpdump -i any -X udp portrange 7401-7500



# enabling encryption
# In terminal 1:
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

# In terminal 2:
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
ros2 run demo_nodes_cpp listener --ros-args --enclave /talker_listener/listener



# displaying encrypeted discovery packets
sudo tcpdump -X -i any udp port 7400



sudo tcpdump -i any -X udp portrange 7401-7500









## setting access control
#modifying permissions.xml
cd ~/sros2_demo/demo_keys/enclaves/talker_listener/talker
mv permissions.p7s permissions.p7s~
mv permissions.xml permissions.xml~
vi permissions.xml


# signin the policy file
openssl smime -sign -text -in permissions.xml -out permissions.p7s \
  --signer permissions_ca.cert.pem \
  -inkey ~/sros2_demo/demo_keys/private/permissions_ca.key.pem

# launching the node
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker \
  --remap chatter:=not_chatter




# using the templates
git clone https://github.com/ros2/sros2.git /tmp/sros2
# talker
ros2 run demo_nodes_cpp talker --ros-args -e /talker_listener/talker
# listener
ros2 run demo_nodes_py listener --ros-args -e /talker_listener/listener

ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener \
  --remap chatter:=not_chatter





## deployment guildelines
# building a deployment scenario
mkdir ~/security_gd_tutorial
cd ~/security_gd_tutorial

# generating Docker image
# Download the Dockerfile
wget https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Advanced/Security/resources/deployment_gd/Dockerfile
# Build the base image
docker build -t ros2_security/deployment_tutorial --build-arg ROS_DISTRO=humble .
# Build the base image
docker build -t ros2_security/deployment_tutorial --build-arg ROS_DISTRO=humble .

# running the example
# Start the example
docker compose -f compose.deployment.yaml up


# examining containers
# Terminal 1
docker exec -it tutorial-listener-1 bash
cd keystore
tree

# Terminal 2
docker exec -it tutorial-talker-1 bash
cd keystore
tree
