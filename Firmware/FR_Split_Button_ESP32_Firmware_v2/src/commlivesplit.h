#ifndef COMMLIVESPLIT_H
#define COMMLIVESPLIT_H

extern unsigned long LastResponseTimestamp;

static void LiveSplitConnect();
int LiveSplitState(int currentState);
static void LiveSplitCommand(int cmd);

#endif //COMMLIVESPLIT_H