#!/bin/bash

# written by Maksym Figat  & Wojciech Dudek
ESC_SEQ="\x1b["
COL_GREEN=$ESC_SEQ"32;01m"
COL_RESET=$ESC_SEQ"39;49;00m"
COL_RED=$ESC_SEQ"31;01m"

GIT_WS_RAPP_NAO_DIR="/home/nao/ws_rapp_nao/src"
GIT_WS_RAPP_APPLICATIONS_DIR="/home/nao/ws_rapp_applications"
GIT_WS_RAPP_API_DIR="/home/nao/ws_rapp_api/src"


WS_RAPP_APPLICATIONS_NAO_DIR="/home/nao/ws_rapp_applications_nao"
WS_RAPP_NAO_DIR="/home/nao/ws_rapp_nao"

HZ_DIRECTORY="/home/nao/ws_rapp_applications/rapp-applications/nao/hz_packages"

ROS_ADDITIONAL_PACKAGES_DIR="/home/nao/ws_ros_additional_packages"
ROS_ADDITIONAL_PACKAGES_SRC_DIR=$ROS_ADDITIONAL_PACKAGES_DIR"/src"

VM_SCRIPTS="/home/nao/scripts"

# If folder doesnt exist
if [ ! -d $GIT_WS_RAPP_NAO_DIR ]; then
	echo -e "$COL_GREEN[OK]$COL_RESET - Creating $GIT_WS_RAPP_NAO_DIR directory."
	mkdir -p $GIT_WS_RAPP_NAO_DIR
fi

# If folder doesnt exist
if [ ! -d $GIT_WS_RAPP_APPLICATIONS_DIR ]; then
	echo -e "$COL_GREEN[OK]$COL_RESET - Creating $GIT_WS_RAPP_APPLICATIONS_DIR directory."
	mkdir -p $GIT_WS_RAPP_APPLICATIONS_DIR
fi

# If folder doesnt exist
if [ ! -d $WS_RAPP_APPLICATIONS_NAO_DIR ]; then
	echo -e "$COL_GREEN[OK]$COL_RESET - Creating $WS_RAPP_APPLICATIONS_NAO_DIR directory."
	mkdir -p $WS_RAPP_APPLICATIONS_NAO_DIR
fi

# If folder doesnt exist
if [ ! -d $GIT_WS_RAPP_API_DIR ]; then
	echo -e "$COL_GREEN[OK]$COL_RESET - Creating $GIT_WS_RAPP_API_DIR directory."
	mkdir -p $GIT_WS_RAPP_API_DIR
fi

# Clonning rapp-robot-nao repository to $GIT_WS_RAPP_NAO_DIR
cd $GIT_WS_RAPP_NAO_DIR
echo -e "$COL_GREEN[OK]$COL_RESET - Clonning rapp-robot-nao repository to $GIT_WS_RAPP_NAO_DIR"
echo -e "$COL_GREEN[OK] - Enter your github login and password $COL_RESET"
git clone -b master https://github.com/rapp-project/rapp-robot-nao.git || { echo -e >&2 "$COL_RED[Error]$COL_RESET - git clone failed with $?"; exit 1; }
	
# Clonning rapp-robot-nao repository to $GIT_WS_RAPP_APPLICATIONS_DIR
cd $GIT_WS_RAPP_APPLICATIONS_DIR
echo -e "$COL_GREEN[OK]$COL_RESET - Clonning rapp-robot-nao repository to $GIT_WS_RAPP_APPLICATIONS_DIR"
echo -e "$COL_GREEN[OK] - Enter your github login and password $COL_RESET"
git clone -b master https://github.com/rapp-project/rapp-applications-nao.git || { echo -e >&2 "$COL_RED[Error]$COL_RESET - git clone failed with $?"; exit 1; }

# Clonning rapp-api repository to $GIT_WS_RAPP_API_DIR
cd $GIT_WS_RAPP_API_DIR
echo -e "$COL_GREEN[OK]$COL_RESET - Clonning rapp-robot-nao repository to $GIT_WS_RAPP_API_DIR"
echo -e "$COL_GREEN[OK] - Enter your github login and password $COL_RESET"
git clone -b robot-api https://github.com/rapp-project/rapp-api.git || { echo -e >&2 "$COL_RED[Error]$COL_RESET - git clone failed with $?"; exit 1; }

# Setting in $WS_RAPP_APPLICATIONS_NAO_DIR a symbolic link to $GIT_WS_RAPP_APPLICATIONS_DIR/rapp-applications/nao/src folder with name src
echo -e "$COL_GREEN[OK]$COL_RESET - Setting in $WS_RAPP_APPLICATIONS_NAO_DIR a symbolic link to $GIT_WS_RAPP_APPLICATIONS_DIR/rapp-applications-nao/nao/src folder with name src"
cd $WS_RAPP_APPLICATIONS_NAO_DIR
ln -s /home/nao/ws_rapp_applications/rapp-applications-nao/nao/src/ src

# If folder doesnt exist
#if [ ! -d $HZ_DIRECTORY/packages ]; then
#	echo -e "$COL_GREEN[OK]$COL_RESET - Creating $HZ_DIRECTORY/packages directory."
#	mkdir -p $HZ_DIRECTORY/packages
#fi

# If folder doesnt exist
if [ ! -d $ROS_ADDITIONAL_PACKAGES_SRC_DIR ]; then 
	echo -e "$COL_GREEN[OK]$COL_RESET - Creates $ROS_ADDITIONAL_PACKAGES_SRC_DIR"
	mkdir -p $ROS_ADDITIONAL_PACKAGES_SRC_DIR
fi

# Adding to system variable PATH - a path to virtual machine scritps
echo -e "$COL_GREEN[OK]$COL_RESET - Adding to system variable PATH - paths to virtual machine scritps"

echo -e "$COL_GREEN[OK]$COL_RESET - Editting ~/.bashrc file"
echo -e "$COL_GREEN[OK]$COL_RESET - Exporting a $GIT_WS_RAPP_NAO_DIR/rapp-robot-nao/scripts/vm_scripts to PATH variable"
export PATH=${PATH}:$GIT_WS_RAPP_NAO_DIR/rapp-robot-nao/scripts/vm_scripts
echo -e "$COL_GREEN[OK]$COL_RESET - Exporting a PATH a path to $GIT_WS_RAPP_APPLICATIONS_DIR/rapp-applications/nao/scripts to PATH variable"
export PATH=${PATH}:$GIT_WS_RAPP_APPLICATIONS_DIR/rapp-applications/nao/scripts
cd ~
/bin/cat <<EOM >".bash_profile"
#!/bin/bash
export PATH=${PATH}:$GIT_WS_RAPP_NAO_DIR/rapp-robot-nao/scripts/vm_scripts
export PATH=${PATH}:$GIT_WS_RAPP_APPLICATIONS_DIR/rapp-applications/nao/scripts
EOM

