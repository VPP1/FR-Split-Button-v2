#ifndef COMMLIVESPLIT_H
#define COMMLIVESPLIT_H

extern unsigned long LastResponseTimestamp;

extern void LiveSplitConnect();
extern int LiveSplitState(int currentState);
extern void LiveSplitCommand(int cmd);

#endif //COMMLIVESPLIT_H