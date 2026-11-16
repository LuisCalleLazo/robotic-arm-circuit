#ifndef Z_AXIS_H
#define Z_AXIS_H

#include "motors.h"

void zAxisInit();
bool zAxisBeginHoming();
HomingStatus zAxisUpdateHoming();
long zAxisSearchSteps();
MoveStart zAxisMoveTo(long target);
MoveStatus zAxisRun();
long zAxisPosition();
long zAxisTarget();
bool zAxisIsReferenced();
void zAxisStop();

#endif
