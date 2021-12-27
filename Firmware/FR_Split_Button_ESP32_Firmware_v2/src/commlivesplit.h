#ifndef COMMLIVESPLIT_H
#define COMMLIVESPLIT_H

extern void LiveSplitConnect();
extern int LiveSplitConnectionStatus(unsigned long timeout);
extern int LiveSplitState(int currentState);
extern void LiveSplitCommand(int cmd);

#endif //COMMLIVESPLIT_H