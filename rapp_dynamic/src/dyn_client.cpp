#include "NaoNavigation.h"

main(int argc, char **argv)
{
	NaoNavigation Nao(argc,argv);
//////
//	Move Nao to next position for increment values (m,m,rad)
///////	
	Nao.moveTo(0.1,0,0);

//////
//	Set Nao velocity moveVel(x,y,theta)
///////	
	Nao.moveVel(1,0,0);


    return 0;
}
