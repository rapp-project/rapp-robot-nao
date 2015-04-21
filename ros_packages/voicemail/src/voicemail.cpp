//#####################
// Maksym Figat
//#####################

#include <rapp_libraries/NaoCommunication.h>
#include <sstream>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


main(int argc, char **argv)
{
   cout<<"Nao Communication - dynamic agent test"<<endl;
   NaoCommunication Nao_comm(argc,argv);
   
   // Robots says
   string str = "Hello";
   Nao_comm.say(str);
   
   // Robot recognizes word from given dictionary
   Nao_comm.say("Setting dictionary");
   string dictionary[] = {"Nao", "Max", "John", "Tom", "Exit"};
   Nao_comm.say("Recognizing a word");
   string recognized_word = Nao_comm.recognizeWord(dictionary,5);
   Nao_comm.say("Recognized word:");
   Nao_comm.say(recognized_word);

   // Robot records a message for a time period
   int time = 10;
   string record_str = "Recording a message for ";
   ostringstream record_tmp;  
   record_tmp<<time;
   record_str = record_str.append(record_tmp.str());
   record_str = record_str.append(" seconds");
   //"Recording a message for a given time
   Nao_comm.say(record_str);
   Nao_comm.record(time);
   Nao_comm.say("Recording stopped");

   //Nao_comm.sendEmail("rapp.nao@gmail.com", "rapp.nao1", "maksym.figat44@gmail.com");
   NaoCommunication::sendEmail("rapp.nao@gmail.com", "password", "maksym.figat44@gmail.com");     

   return 0;
}
