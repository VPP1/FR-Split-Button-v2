#ifndef COMMSPEEDCONTROL_H
#define COMMSPEEDCONTROL_H

extern void SpeedCtrlConnect();
extern int SpeedCtrlConnectionStatus(unsigned long timeout);
extern int SpeedCtrlState();
extern void SpeedCtrlCommand(int cmd);

#endif //COMMSPEEDCONTROL_H