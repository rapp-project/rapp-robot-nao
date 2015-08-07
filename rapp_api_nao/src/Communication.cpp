#include <rapp/robot/Communication.hpp>
#include "CommunicationImpl.hpp"

namespace rapp {
namespace robot {

Communication::Communication(int argc, char * argv[]) {
	pimpl = new CommunicationImpl(argc, argv);
}

Communication::~Communication() {
	delete pimpl;
}

bool Communication::playAudio(const std::string & file_path, double position, double volume, double balance, bool play_in_loop) {
	pimpl->playAudio(file_path, position, volume, balance, play_in_loop);
}

bool Communication::textToSpeech(const std::string & str) {
	pimpl->textToSpeech(str);
}

bool Communication::textToSpeech(const std::string & str, Language language) {
	pimpl->textToSpeech(str, language);
}

std::string wordSpotting(std::string dictionary[], int size){
	pimpl->wordSpotting(dictionary, size);
}

std::string captureAudio(int time){
	pimpl->captureAudio(time);
}

std::string captureAudio(std::string & file_path, float waiting_time, int microphone_energy){
	pimpl->captureAudio(file_path, waiting_time, microphone_energy);
}

int microphoneEnergy(std::string & name){
	pimpl->microphoneEnergy(name);
}

void voiceRecord(bool startRecording, std::vector< std::vector<unsigned char> > &audio_buffer_vector){
	pimpl->voiceRecord(startRecording, audio_buffer_vector);
}

}
}

