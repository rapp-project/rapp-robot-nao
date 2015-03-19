//#####################
// written by Jan Figat & Wojciech Dudek & Maksym Figat
//#####################

#include "NaoCommunication.h"

main(int argc, char **argv)
{
   cout<<"Nao Communication - dynamic agent test"<<endl;
   NaoCommunication Nao_comm(argc,argv);
   
   // Robots says
   string str = "Hello";
   Nao_comm.say(str);
   
   // Robot recognizes word from given dictionary
   string dictionary[] = {"Nao", "Max", "John", "Tom", "Exit"};
   Nao_comm.recognizeWord(dictionary,5);

   // Robot records a message for a time period
   int time = 10;
   Nao_comm.record(time);
   return 0;
}
