sudo apt update 
sudo apt -y upgrade

sudo raspi-config
# enable i2c and serial conection in interface options

# setup ssh
sudo apt-get install -y openssh-client
sudo apt-get install -y openssh-server

########### install Docker #########################

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# install docker
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

xhost +local:docker
sudo usermod -aG docker $USER

########### configure git ####################

# configure git 
sudo apt install -y git-all
git config --global user.email "tommaso.scudeletti@mail.polimi.it"
git config --global user.name "ScudeT"

cd
git clone https://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@github.com/ScudeT/Maui.git

#### configure gpsd ######
sudo apt-get update
sudo apt-get  install gpsd gpsd-clients
sudo nano /etc/default/gpsd
# in the file 
# START_DAEMON="true"
# USBAUTO="true"
# DEVICES="/dev/ttyUSB0"
# GPSD_OPTIONS="-n"

echo "cd Maui" >> /home/${USERNAME}/.bashrc
cd Maui/docker/raspi_ros2
docker compose up -d


### CAMERA SETUP ####
pip3 install opencv-python flask picamera2 --break-system-packages
