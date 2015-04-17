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
   string str = "Hello world";
   Nao_comm.say(str);
   
   return 0;
}
