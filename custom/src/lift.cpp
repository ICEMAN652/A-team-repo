#include "vex.h"
#include "../include/robot-config.h"
#include "../include/lift.h"

void liftTo(double deg, bool blocking, int timeout_ms) {
  lift.spinToPosition(deg, degrees, false);
  if (!blocking) return;

  // wait for the lift, but give up after timeout_ms so a stuck lift can't hang the auton
  int elapsed = 0;
  while (!lift.isDone() && elapsed < timeout_ms) {
    wait(10, msec);
    elapsed += 10;
  }
}
