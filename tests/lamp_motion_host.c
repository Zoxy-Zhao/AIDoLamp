#include <stdio.h>
#include "lamp_motion.h"
static void emit(unsigned channel,float angle) { printf("%u:%.3f ",channel,angle); }
int main(void) {
    char line[256];
    while(fgets(line,sizeof(line),stdin)) { int result=lamp_motion(line,emit); printf("result=%d\n",result); }
    return 0;
}
