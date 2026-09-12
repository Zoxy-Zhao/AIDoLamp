#ifndef LAMP_MOTION_H
#define LAMP_MOTION_H
/* Shared by contact.c and host tests. Four arm channels + separate gripper.
 * Example gripper PWM channel/angle range: calibrate before physical use. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#ifndef LAMP_GRIPPER_CHANNEL
#define LAMP_GRIPPER_CHANNEL 4
#endif
#if LAMP_GRIPPER_CHANNEL < 4 || LAMP_GRIPPER_CHANNEL > 15
#error Gripper channel must not overlap arm channels 0..3
#endif
#ifndef LAMP_GRIPPER_MIN
#define LAMP_GRIPPER_MIN 0.0f
#endif
#ifndef LAMP_GRIPPER_MAX
#define LAMP_GRIPPER_MAX 180.0f
#endif
/* 1 accepted, -1 malformed motion, 0 another command. No partial actuation. */
static int lamp_motion(const char *line, void (*emit)(unsigned, float)) {
    const float lo[4]={-135,-90,0,-90}, hi[4]={135,90,150,40};
    const char *p=line; float angles[4]; unsigned n=0, gripper=0;
    while (isspace((unsigned char)*p)) ++p;
    if (!strncmp(p,"Alldro",6) && (!p[6] || isspace((unsigned char)p[6]))) { p+=6; n=4; }
    else if (!strncmp(p,"GRIP",4) && (!p[4] || isspace((unsigned char)p[4]))) { p+=4; n=1; gripper=1; }
    else return 0;
    for (unsigned i=0;i<n;++i) {
        char *end;
        while (isspace((unsigned char)*p)) ++p;
        double v=strtod(p,&end);
        if (p==end || !isfinite(v) || (*end && !isspace((unsigned char)*end))) return -1;
        if (v < (gripper?LAMP_GRIPPER_MIN:lo[i]) || v > (gripper?LAMP_GRIPPER_MAX:hi[i])) return -1;
        angles[i]=(float)v; p=end;
    }
    while (isspace((unsigned char)*p)) ++p;
    if (*p) return -1;
    for (unsigned i=0;i<n;++i) emit(gripper?LAMP_GRIPPER_CHANNEL:i,angles[i]);
    return 1;
}
#endif
