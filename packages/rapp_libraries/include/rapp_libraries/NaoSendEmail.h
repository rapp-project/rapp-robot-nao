#include <iostream>
#include <sstream>
#include <vector>
#include <map>

#include "vmime/vmime.hpp"
#include "vmime/platforms/posix/posixHandler.hpp"

#define AUDIO_PATH "/home/max/fun/sounds/sound.ogg"
#define IMAGE_PATH "/home/max/fun/photo/domi.jpg"

// Global session object
//static vmime::ref <vmime::net::session> g_session
//	= vmime::create <vmime::net::session>();



class SendEmailClass
{

public:
	

	static void sendMessage(std::string login, std::string password, std::string sendTo);

};

