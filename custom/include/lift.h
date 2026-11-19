#pragma once

// Moves the lift to deg degrees.
// - blocking: true waits for the lift to get there before going on, false keeps going right away
// - timeout_ms: when blocking, gives up after this long so a stuck lift can't freeze the auton
// e.g. liftTo(400, true, 800);  or  liftTo(0, false);
void liftTo(double deg, bool blocking, int timeout_ms = 1000);
