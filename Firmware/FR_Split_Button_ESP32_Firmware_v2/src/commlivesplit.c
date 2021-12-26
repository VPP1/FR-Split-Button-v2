
#include "commlivesplit.h"

#include <sys/socket.h>

//LiveSplit Server IP/Port
#define LIVESPLIT_IP "192.168.0.11"
#define LIVESPLIT_PORT 16834

//LiveSplit Server command messages
#define LS_CMD_GET_STATE "getcurrenttimerphase\r\n"
#define LS_CMD_START_OR_SPLIT "startorsplit\r\n"
#define LS_CMD_PAUSE "pause\r\n"
#define LS_CMD_RESUME "resume\r\n"
#define LS_CMD_RESET "reset\r\n"

//LiveSplit Server timer status messages
#define LS_MSG_STATE_NOT_RUNNING "NotRunning"
#define LS_MSG_STATE_RUNNING "Running"
#define LS_MSG_STATE_ENDED "Ended"
#define LS_MSG_STATE_PAUSED "Paused"

//Timer control command enumeration
#define TIMER_CMD_GET_STATE 0
#define TIMER_CMD_START_OR_SPLIT 1
#define TIMER_CMD_STOP 2
#define TIMER_CMD_PAUSE 3
#define TIMER_CMD_RESUME 4
#define TIMER_CMD_RESET 5


//Main state variable
//0 = default, not connected/error
//1 = not running
//2 = timer running
//3 = timer finished (ended)
//4 = timer paused
#define TIMER_STATE_DEFAULT 0
#define TIMER_STATE_STANDBY 1
#define TIMER_STATE_RUNNING 2
#define TIMER_STATE_FINISHED 3
#define TIMER_STATE_PAUSED 4

//Network socket
static int Socket = 0;

//Last response timestamp
unsigned long LastResponseTimestamp = 0;


//Event handler for ethernet events, sets MAC-address upon connecting
//Connect to livesplit server. Keep retrying until a successfull connection is made
void LiveSplitConnect()
{
    //Create a socket
    Socket = socket(AF_INET, SOCK_STREAM, 0);
    
    //Specify IP-address for the socket
    struct sockaddr_in serverAddr =
    {
        .sin_family = AF_INET,
        .sin_port = htons(LIVESPLIT_PORT),
        .sin_addr.s_addr = inet_addr(LIVESPLIT_IP)
    };

    //Connect to the server (LiveSplit)
    //Only print out error message if first attempt does not go through
    int connectionStatus = -1;  //connect returns -1 on failed attempt
    int firstAttempt = 1;

    while(connectionStatus == -1)
    {
        if(firstAttempt == 0)
        {
            printf("Error connecting to LiveSplit! Retrying...\n");
        }

        firstAttempt = 0;
        connectionStatus = connect(Socket, (struct sockaddr *) &serverAddr, sizeof(serverAddr));
    }

    printf("Connected to LiveSplit.\n");
}


//Reads a response from LiveSplit server and determines the timer state
int LiveSplitState(int currentState)
{
    int state = currentState;   //Init to current state, only update if data is available
    char serverResponse[256];

    //Setup socket monitoring, wait for 20000us before timing out
    int dataAvailable = 0;
    fd_set readFds;
    struct timeval timeout =
    {
        .tv_sec = 0,
        .tv_usec = 20000
    };

    FD_ZERO(&readFds);
    FD_SET(Socket, &readFds);

    dataAvailable = select(Socket+1, &readFds, NULL, NULL, &timeout);

    //If data is available, read it and determine the timer state
    if(dataAvailable != 0)
    {
        ssize_t len = recv(Socket, &serverResponse, sizeof(serverResponse), 0);

        if(len > 0)
        {
            LastResponseTimestamp = xTaskGetTickCount();    //Save the tick timestamp for connection status monitoring
        }

        //Check if the predefined responses are included in the server response
        if(strstr(serverResponse, LS_MSG_STATE_NOT_RUNNING) != NULL)
        {
            state = TIMER_STATE_STANDBY;
        }
        else if(strstr(serverResponse, LS_MSG_STATE_RUNNING) != NULL)
        {
            state = TIMER_STATE_RUNNING;
        }
        else if(strstr(serverResponse, LS_MSG_STATE_ENDED) != NULL)
        {
            state = TIMER_STATE_FINISHED;
        }
        else if(strstr(serverResponse, LS_MSG_STATE_PAUSED) != NULL)
        {
            state = TIMER_STATE_PAUSED;
        }
    }
    
    return state;
}

//Send string command to LiveSplit server
void LiveSplitCommand(int cmd)
{
    switch (cmd)
    {
        case TIMER_CMD_GET_STATE:
            send(Socket, LS_CMD_GET_STATE, strlen(LS_CMD_GET_STATE), 0);
            break;
        case TIMER_CMD_START_OR_SPLIT:
            send(Socket, LS_CMD_START_OR_SPLIT, strlen(LS_CMD_START_OR_SPLIT), 0);
            break;
        case TIMER_CMD_PAUSE:
            send(Socket, LS_CMD_PAUSE, strlen(LS_CMD_PAUSE), 0);
            break;
        case TIMER_CMD_RESUME:
            send(Socket, LS_CMD_RESUME, strlen(LS_CMD_RESUME), 0);
            break;
        case TIMER_CMD_RESET:
            send(Socket, LS_CMD_RESET, strlen(LS_CMD_RESET), 0);
            break;
        default:
            break;
    }
}
