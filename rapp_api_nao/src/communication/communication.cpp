#include <rapp/robot/communication/communication.hpp>
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
	bool success;
	success = pimpl->playAudio(file_path, position, volume, balance, play_in_loop);
	return success;
}

//bool Communication::textToSpeech(const std::string & str) {
//	pimpl->textToSpeech(str);
//}

bool Communication::textToSpeech(const std::string & str, Language language) {
	bool success;
	success = pimpl->textToSpeech(str, language);
	return success;
}

std::string Communication::wordSpotting(std::string dictionary[], int size){
	std::string word;
	word = pimpl->wordSpotting(dictionary, size);
	return word;
}

std::string Communication::captureAudio(int time){
	std::string message;
	message = pimpl->captureAudio(time);
	return message;
}

std::string Communication::captureAudio(std::string & file_path, float waiting_time, int microphone_energy){
	std::string message;
	message = pimpl->captureAudio(file_path, waiting_time, microphone_energy);
	return message;
}

int Communication::microphoneEnergy(std::string & name){
	int energy;
	energy = pimpl->microphoneEnergy(name);
	return energy;
}

void Communication::voiceRecord(bool startRecording, std::vector< std::vector<unsigned char> > &audio_buffer_vector){
	pimpl->voiceRecord(startRecording, audio_buffer_vector);
}

} // namespace robot
} // namespace rapp
