#include <rapp/robot/Communication.hpp>
#include "CommunicationImpl.hpp"

namespace rapp {
namespace robot {
	
bool Communication::playAudio(const std::string & file_path, double position, double volume, double balance, bool play_in_loop) {
	pimpl->playAudio(file_path, position, volume, balance, play_in_loop);
}

bool Communication::textToSpeech(const std::string & str, Language language) {
	pimpl->textToSpeech(str, language);
}

}
}

