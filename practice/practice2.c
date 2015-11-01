#include <stdio.h>

int main(void)
{
  char *str = "good bay";

  str [7] = 'y';
  str [6] = 'e';


  printf("%s\n", str);

  return(0);

}
