	#include <rapp_api/NaoDAStatus.h>


	// Class constructor that sends status of dynamic agent to core agent.
	NaoDAStatus::NaoDAStatus(int argc,char **argv){
		ros::init(argc, argv,"NaoDynamicAgentStatus");
		n = new ros::NodeHandle();
		sendDAStatus("Init");
	}

	NaoDAStatus::~NaoDAStatus()
	{
		sendDAStatus("Finished");
	}

	// Function from Rapp API that calls say service from core agent on Nao robot. Robot says a given sentence.
	void NaoDAStatus::sendDAStatus(string str){	

		client_status = n->serviceClient<rapp_core_agent::DynamicAgentStatus>("dynamic_agent_status");
		rapp_core_agent::DynamicAgentStatus srv;
		srv.request.da_status = str;
		srv.request.pid=getpid();
		cout<<srv.request.pid<<endl;
		if (client_status.call(srv))
		{
			ROS_INFO("[Dynamic agent] - Calling service DynamicAgentStatus - sends status of DA to Core Agent");
			ca_status=srv.response.ca_status;
			cout<<"[Dynamic agent] - status received from core agent: " <<ca_status<<endl;	
			
			return;
		}
		else
		{
			ROS_ERROR("Failed to call service DynamicAgentStatus");
			ca_status="Not connected with CA";
			cout<<"[Dynamic agent] - status received from core agent: " <<ca_status<<endl;
		}

		return;
	}

	string NaoDAStatus::getCAStatus()
	{
		return ca_status;
	}







