Initial requirements
====================

It is assumed that the below are properly installed:

- ROS Indigo distribution (http://wiki.ros.org/indigo)
- `rosbridge_suite` package (http://wiki.ros.org/rosbridge_suite)
- the latest versions of HOP and Bigloo for RAPP partners
  (ftp://ftp-sop.inria.fr/indes/rapp/hop/2014-09-05)


Overview
========

This demo shows how to use HOP and ROS to implement a basic
functionality of a core agent: downloading and executing applications
(HOP weblets) from the cloud (RAPP store). It requires two instances
of HOP server: one on the cloud, and one on the robot. The HOP server
on the cloud is responsible for delivering the applications to the
robot. The HOP server on the robot is responsible for downloading and
installing the applications from the cloud. The third component of the
demo is a ROS node that runs on the robot and interacts with the
user, asking for applications to be downloaded. It sends the name of
the requested application to the HOP server on the robot. Once the
application is downloaded from the cloud and installed on the robot,
the installation path is sent back to the ROS node that finally
triggers execution of the application.

The all three components of the demo and communication links between
them are depicted below:

    HOP (cloud) <---> HOP (robot) <---> ROS node (robot)



Setting up the demo and further details
=======================================

The instructions for setting up the demo are given separately for each
of the components. Although not always required to do so, it is best
to follow the instructions' order.


HOP (cloud)
-----------

Normally, this HOP server runs on the cloud, but for the purpose of
the demo, run it on the local machine (on port 9999):

    $ hop -p 9999


HOP (robot)
-----------

The second HOP server is responsible for downloading and installing
HOP weblets. This server communicates with the ROS node using
ROSbridge, so prior to running this server, ROS and `rosbridge_server`
must be running (see the `rosbridge` demo provided by INRIA;
https://github.com/rapp-project/hop_samples.git).

In a new terminal, run:

    $ roslaunch rosbridge_server rosbridge_websocket.launch

Once this is done, both the ROS (ROS master) and the ROSbridge server
should be running.

Now, in a new terminal, go to the `hop` subdirectory of the demo and
start the HOP server (on port 8080 by default) with
`store_interaction.js` script:

    $ hop store_interaction.js

This script adds two active ROS topics:

- `/rapp/store_interaction/request`, and
- `/rapp/store_interaction/response`

You can check it, running in a new terminal: `rostopic list`.

Both topics are of ROS type `std_msgs/String`. To download and install
a given weblet, one can publish a message with its name on the topic
`/rapp/store_interaction/request`. Once it is installed, a message
with the path to the weblet's directory is published on the topic
`/rapp/store_interaction/response`. On failure, the published message
contains empty string.


ROS node (robot)
----------------

The ROS node performs an interaction with a user. Normally, such an
interaction is done through the robot's sensors/effectors (e.g. NAO's
microphone and speaker), but for the purpose of the demo, the Unix
terminal is used. The user is first asked to enter the name of an
application (HOP weblet) to be downloaded and installed. This name
is then published on the topic `/rapp/store_interaction/request`. At
the same time, the ROS node is listening on the topic
`/rapp/store_interaction/response`. When the path to the application's
directory is received, the ROS node executes the shell command
`sh run` in that directory. The shell script `run` is assumed to be
a part of the installed application.

To build and setup the ROS node, open a new terminal, go to the `ros`
subdirectory (ros workspace) of the demo and run:

    $ catkin_make

    $ source devel/setup.bash

Now, you can run the ROS node:

    $ rosrun rapp core_agent

In the `hop` subdirectory, the demo contains a simple HOP weblet
called `hello-1.0.0.hz`. The `run` script inside this hz package
prints the text `This is your RAPP application speaking!` to the
terminal.


Example user interaction
========================

    $ rosrun rapp core_agent
    Enter the name of a RAPP application (or 'q' to quit): test-0.0.0.hz
    Downloading...
    Failed
    Enter the name of a RAPP application (or 'q' to quit): hello-1.0.0.hz
    Downloading...
    Installed in ~/.config/hop/weblets/hello
    Starting...
    This is your RAPP application speaking!
    Enter the name of a RAPP application (or 'q' to quit): q
    $
