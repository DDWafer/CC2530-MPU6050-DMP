
#include "net_ID.h" 

//typedef unsigned short int u16;
extern void ini(void);
extern void osal_start_system(void);
unsigned short int zgConfigPANID=net_ID;
int main( void )
{
  ini();
  osal_start_system(); // No Return from here

  return 0;  // Shouldn't get here.
} // main()
