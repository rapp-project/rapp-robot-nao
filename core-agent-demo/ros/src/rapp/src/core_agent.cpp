
// core_agent.cpp
// An example of RAPP core agent functionality: downloading 
// and running applications from the cloud (RAPP store)
//
// Marcin Szlenk <m.szlenk@elka.pw.edu.pl>
// Maksym Figat <maksym.figat44@gmail.com>
// Copyright 2014 RAPP

#include "ros/ros.h"
#include "std_msgs/String.h"

// Name of the ROS topic for receiving requests.
#define REQUEST_TOPIC "/rapp/store_interaction/request"
// Name of the ROS topic for publishing requests.
#define RESPONSE_TOPIC "/rapp/store_interaction/response"

// Ask for the name of an application to download and publish the
// request to HOP, where:
//
// p - a pointer to the ROS publisher object; must be a non-NULL
//     on the first call to the function and is ignored on the
//     subsequent calls.
void sendRequest(ros::Publisher *p = NULL)
{
    static ros::Publisher& pub = *p;

    std::string name;

    std::cout << "Enter the name of a RAPP application (or \'q' to quit): ";
    std::cin >> name;

    if(name == "q")
    {
        // Shut down the node.
        ros::shutdown();
    }
    else
    {
        std_msgs::String msg;

        // Publish a message with the application name.
        msg.data = name;
        pub.publish(msg);

        std::cout << "Downloading...\n";
    }
}

// A callback function. Executes each time a new response
// message from HOP arrives.
void responseReceived(const std_msgs::String& msg)
{
    // Response from HOP.
    std::string path = msg.data;

    if(path.empty())
    {
        std::cout << "\e[1m\e[31m" // bold and red font
            << "Failed"
            << "\e[0m\n"; // normal print mode
    }
    else
    {
        std::cout << "\e[1m\e[31m"
            << "Installed in "
            << path << "\e[0m\n"; 
        
        // Build the path to the 'run' script of the application.
        path.append("/run");

        std::cout << "Starting...\n";

        // Run the script.
        path = std::string("sh ").append(path);
        system(path.c_str());
    }

    // Ask for the next application's name.
    sendRequest();
}

int main(int argc, char **argv)
{
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "core_agent");
    ros::NodeHandle nh;

    // Create a subscriber object.
    ros::Subscriber sub =
        nh.subscribe(RESPONSE_TOPIC, 100, &responseReceived);
    
    // Create a publisher object.
    ros::Publisher pub =
        nh.advertise<std_msgs::String>(REQUEST_TOPIC, 100);

    // Send the initial request.
    sendRequest(&pub);

    // Let ROS take over and execute callbacks.
    ros::spin();
}
