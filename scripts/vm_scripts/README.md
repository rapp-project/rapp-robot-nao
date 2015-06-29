VIRTUAL MACHINE SCRIPTS
========================================================
TODO - what is virtual machine and why scripts are needed?

While writing a program for NAO user is utilising a cross compilation method. Source code has to be compiled and built on virtual machine and then sent to NAO robot.

List of scripts
------------
* vm_send_core_agent_packages_to_nao.sh - synchronizes workspaces from virtual machine with the Nao robot with the given IP address (ws_rapp_nao - built core agent and an empty template for workspace ws_rapp_applications_nao). It also sends data structure to launch voicemail. 
* vm_send_ros_libraries_to_nao.sh - synchronizes workspaces from virtual machine with the Nao robot with the given IP address (ws_ros - built ROS, ws_ros_additional_packages - built additional external packages/libraries that are needed to be downloaded by NAO) ready to use on NAO robot.
* vm_update_structure.sh - updates (using synchronization) the folder structure on the Nao robot from folder structure on virtual machine.
* vm_compile_additional_packages.sh - builds packages from ws_ros workspace (if not compiled). Downloads external libraries (gsasl, vmime), needed ROS packages (bond_core, cmake_modules, image_common, nodelet_core, vision_opencv, rosbridge_suite) and installs/builds them into /home/nao/ws_ros_additional_packages/install_isolated/
* vm_compile_rapp_core_agent.sh - builds ROS workspace ws_rapp_nao
* vm_compile_rapp_dynamic_agents.sh - builds ROS workspace ws_rapp_applications_nao
* vm_compile_rapp_workspaces.sh - builds ROS packages from ws_rapp_nao and ws_rapp_applications_nao
* vm_preparation_to_build_packages.sh - clones github repositories (https://github.com/rapp-project/rapp-robot-nao.git and https://github.com/rapp-project/rapp-applications.git from master branch), creates correct file structure on virtual machine.
* vm_ros_tutorial.sh - script that can be useful for passing a (ROS Installation on VM for Nao) Tutorial. Please note that if you get a correct virtual machine image than you get there a ROS packages ready to build.
