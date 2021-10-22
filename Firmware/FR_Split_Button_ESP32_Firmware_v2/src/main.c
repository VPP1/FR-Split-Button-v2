
/*
TODO:
Expand the usb slot
If split/pause button is held down long enough, a message is sent on the depress too

Changed connection status to a global variable so it can be monitored
Project did not build anymore. Missing CMake dir and something else
Tried to update PIO, did not help. Updating introduced some intellisense errors
which were not present before....

TCP/IP communication:
- untested: initialization
- untested: sending commands to livesplit & recieving timer status
- not done: replacing test things in button interrupts to actually sending livesplit commands

Notes:
Add/change to sdkconfig:
CONFIG_ETH_RMII_CLK_OUTPUT=y
CONFIG_ETH_RMII_CLK_OUT_GPIO =17

How is multiple buttons during a race handled? Probably some kind of a custom
command to a custom timer which is done via LiveSplit Core or something similar?
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "tcpip_adapter.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "olimex_ethernet.h"

//For delays
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


//I/O
#define GPIO_BTN_PAUSE 14     //Input_pullup (UEXT)
#define GPIO_BTN_SPLIT 2      //Input_pullup (UEXT)
#define GPIO_LED_SPLIT 15     //Output (UEXT)
#define PIN_PHY_POWER 12      //Ethernet physical layer enable -output
#define PIN_ETH_MDC 23
#define PIN_ETH_MDIO 18

//LED dynamization parameters, in milliseconds
#define TIME_FADE_STANDBY 4000
#define TIME_FADE_TIMER_FINISHED 500
#define TIME_FADE_TIMER_PAUSED 50

//Max led brightness
//Since we are using 5kHz PWM freqeuncy defined in ledc_timer_config_t,
//it means we have 13-bit resolution duty variable. 
//0% - 100% duty => 0 - 8192 duty variable
#define LED_DUTY_MAX 8192

//Local address
#define LOCAL_IP "192.168.0.21"
#define SUBNETMASK "255.255.255.0"

//LiveSplit Server IP/Port
#define LIVESPLIT_IP "192.168.0.11"
#define LIVESPLIT_PORT 16834

//LiveSplit Server command/status messages
/*
#define CMD_SPLIT "startorsplit\r\n"
#define CMD_PAUSE "pause\r\n"
#define CMD_GET_STATE "getcurrenttimerphase\r\n"
*/

#define CMD_SPLIT "startorsplit<EOF>"
#define CMD_PAUSE "pause<EOF>"
#define CMD_GET_STATE "getcurrenttimerphase<EOF>"

#define MSG_STATE_NOT_RUNNING "NotRunning"
#define MSG_STATE_RUNNING "Running"
#define MSG_STATE_ENDED "Ended"
#define MSG_STATE_PAUSED "Paused"

//Main state variable
//0 = default, not connected/error
//1 = not running
//2 = timer running
//3 = timer finished (ended)
//4 = timer paused
const static int TimerDefault = 0;
const static int TimerStandby = 1;
const static int TimerRunning = 2;
const static int TimerFinished = 3;
const static int TimerPaused = 4;

static int TimerState = TimerDefault;

//PWM phase flag
//1 = at max duty
//0 = at min duty (0)
static int PWMPhase = 0;

//Button falling edge debouncing variables
//Filters out extra interrupt triggers upon pressing a button
//Without filtering, the interrupts can trigger up to 20 times when pressing a button
//Define time in milliseconds at pdMS_TO_TICKS()
unsigned long TrgDebouncingLimit = pdMS_TO_TICKS(250);
unsigned long TrgTickStampSplit = 0;
unsigned long TrgPrevTickStampSplit = 0;
unsigned long TrgTickStampPause = 0;
unsigned long TrgPrevTickStampPause = 0;

//Network socket
int NetworkSocket = 0;

//Command sending flags
int SendSplit = 0;
int SendPause = 0;


int CStatus = -9999;

//////////////////////////////CONFIGURATIONS//////////////////////////////
//PWM configuration
const static ledc_timer_config_t SplitLEDPWMConfig =
{
    .duty_resolution = LEDC_TIMER_13_BIT,   //Resolution of PWM duty
    .freq_hz = 5000,                        //Frequency of PWM signal
    .speed_mode = LEDC_HIGH_SPEED_MODE,     //Timer mode
    .timer_num = LEDC_TIMER_0,              //Timer index
    .clk_cfg = LEDC_AUTO_CLK,               //Auto select the source clock 
};

const static ledc_channel_config_t SplitLEDChannelConfig =
{
    .channel    = LEDC_CHANNEL_0,
    .duty       = 0,
    .gpio_num   = GPIO_LED_SPLIT,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .hpoint     = 0,
    .timer_sel  = LEDC_TIMER_0
    //.intr_type = LEDC_INTR_FADE_END
};

//Button I/O config - input pullup
//Pause button
const static gpio_config_t PauseBtnConfig =
{
    .intr_type = GPIO_PIN_INTR_NEGEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL<<GPIO_BTN_PAUSE),
    .pull_up_en = GPIO_PULLUP_ENABLE
};

//Split button
const static gpio_config_t SplitBtnConfig =
{
    .intr_type = GPIO_PIN_INTR_NEGEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL<<GPIO_BTN_SPLIT),
    .pull_up_en = GPIO_PULLUP_ENABLE
};


//////////////////////////////ETHERNET COMMUNICATION//////////////////////////////
//Event handler for ethernet events, sets MAC-address upon connecting
//Connect to livesplit server. Keep retrying until a successfull connection is made
/*
static void LiveSplitConnect()
{
    //Create a socket
    NetworkSocket = socket(AF_INET, SOCK_STREAM, 0);
    
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
            vTaskDelay(pdMS_TO_TICKS(2000));
            printf("Error connecting to LiveSplit! Retrying...\n");
        }

        firstAttempt = 0;
        connectionStatus = connect(NetworkSocket, (struct sockaddr *) &serverAddr, sizeof(serverAddr));
    }

    printf("Connected to LiveSplit.\n");
}
*/

static void LiveSplitConnect()
{  
    //Specify IP-address for the socket
    struct sockaddr_in serverAddr =
    {
        .sin_family = AF_INET,
        .sin_port = htons(LIVESPLIT_PORT),
        .sin_addr.s_addr = inet_addr(LIVESPLIT_IP)
    };

    //Create a socket
    //NetworkSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int networkSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if(networkSocket < 0)
    {
        printf("Unable to create socket.\n");
    }

    //Connect to the server (LiveSplit)
    //Only print out error message if first attempt does not go through
    printf("Trying to connect to %d\n", serverAddr.sin_addr.s_addr);

    //int connectionStatus = connect(networkSocket, (struct sockaddr *) &serverAddr, sizeof(struct sockaddr));
    CStatus = connect(networkSocket, (struct sockaddr *) &serverAddr, sizeof(struct sockaddr));
    int firstAttempt = 1;
}


//Sends a command to LiveSplit server
void LiveSplitQuery(int socket, char command[256])
{
    char msg[256];
    strcpy(msg, command);

    printf("Sending CMD: %s\n", msg);

    send(socket, msg, sizeof(msg), 0);
}


//Reads a response from LiveSplit server and determines the timer state
int LiveSplitState(int socket, int currentState)
{
    int status = 0;
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
    FD_SET(socket, &readFds);

    dataAvailable = select(socket+1, &readFds, NULL, NULL, &timeout);

    //If no data is available, set current status
    //If data is available, read it and determine the timer status
    if(dataAvailable == 0)
    {
        status = currentState;
    }
    else
    {
        recv(socket, &serverResponse, sizeof(serverResponse), 0);

        if(strcmp(serverResponse, MSG_STATE_NOT_RUNNING) == 0)
        {
            status = TimerStandby;
        }
        else if(strcmp(serverResponse, MSG_STATE_RUNNING) == 0)
        {
            status = TimerRunning;
        }
        else if(strcmp(serverResponse, MSG_STATE_ENDED) == 0)
        {
            status = TimerFinished;
        }
        else if(strcmp(serverResponse, MSG_STATE_PAUSED) == 0)
        {
            status = TimerPaused;
        }
    }
    
    return status;
}


void EthernetEvent(void *arg, esp_event_base_t eventBase, int32_t eventID, void *eventData)
{
    uint8_t macAddr[6] = {0};
    esp_eth_handle_t ethHandle = *(esp_eth_handle_t *)eventData;

    tcpip_adapter_ip_info_t ipInfo =
    {
        .ip.addr = ipaddr_addr(LOCAL_IP),
        .gw.addr = ipaddr_addr("192.168.0.0"),
        .netmask.addr = ipaddr_addr(SUBNETMASK)
    };

    if(eventID == ETHERNET_EVENT_CONNECTED)
    {
        esp_eth_ioctl(ethHandle, ETH_CMD_G_MAC_ADDR, macAddr);

        ESP_ERROR_CHECK(tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_ETH)); 
        ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_ETH, &ipInfo));  //Set static IP

        printf("Ethernet MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
                 macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
    }
}


//Event handler for getting an IP address
static void GotIPEvent(void *arg, esp_event_base_t eventBase,
                                 int32_t eventID, void *eventData)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) eventData;
    const tcpip_adapter_ip_info_t *ip_info = &event->ip_info;

    printf("Got IP: " IPSTR, IP2STR(&ip_info->ip));
    printf("\n");

    LiveSplitConnect();
}


//////////////////////////////LED CONTROL//////////////////////////////
//Set LED fading towards the wanted duty cycle
void LEDFade(ledc_channel_config_t led, uint32_t duty, int fadeTime)
{
    ledc_set_fade_with_time(led.speed_mode, led.channel, duty, fadeTime);
    ledc_fade_start(led.speed_mode, led.channel, LEDC_FADE_NO_WAIT);
}

//Checks if the LED has reached max duty
//Another option would have been to attach an interrupt to the fade function completing,
//but it crashes the CPU and I couldn't figure out how to fix it.
int CheckLEDFadeProgress(ledc_channel_config_t cfg, uint32_t maxDuty)
{
    uint32_t ledDuty = ledc_get_duty(cfg.speed_mode, cfg.channel);

    if((ledDuty >= maxDuty) || (ledDuty == 0))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//Called periodicaly whenever the LED duty has reached bottom or top limit, or when the LED is in a static state
void UpdateLEDState()
{
    //Determine next duty value
    uint32_t duty;
   
    PWMPhase = !PWMPhase; //Phase has reached the other end

    if(PWMPhase == 1)
    {
        duty = 0; //At high phase, start going towards 0
    }
    else
    {
        duty = LED_DUTY_MAX; //At low phase, start going towards max value
    }
    
    //Set the led fading at a speed defined for each timer state, or set it a static on/off
    switch(TimerState)
    {
        case TimerStandby:
            LEDFade(SplitLEDChannelConfig, duty, TIME_FADE_STANDBY);
            break;
        case TimerRunning:
            ledc_set_duty(SplitLEDChannelConfig.speed_mode, SplitLEDChannelConfig.channel, LED_DUTY_MAX);
            ledc_update_duty(SplitLEDChannelConfig.speed_mode, SplitLEDChannelConfig.channel);
            PWMPhase = 1;
            break;
        case TimerFinished:
            LEDFade(SplitLEDChannelConfig, duty, TIME_FADE_TIMER_FINISHED);
            break;
        case TimerPaused:
            LEDFade(SplitLEDChannelConfig, duty, TIME_FADE_TIMER_PAUSED);
            break;
        default:
            ledc_set_duty(SplitLEDChannelConfig.speed_mode, SplitLEDChannelConfig.channel, 0);
            ledc_update_duty(SplitLEDChannelConfig.speed_mode, SplitLEDChannelConfig.channel);
            PWMPhase = 0;
            break;
    }
}


//////////////////////////////INTERRUPTS (BUTTONS)//////////////////////////////
//Checks if enough ticks have passed since the last interrupt
int FallingEdgeDebouncing(unsigned long *tickStamp, unsigned long *prevTickStamp, unsigned long debouncingLimit)
{
    *tickStamp = xTaskGetTickCount();

    if((*tickStamp - debouncingLimit) > *prevTickStamp)
    {
        *prevTickStamp = *tickStamp;
        return 1;
    }
    else
    {
        return 0;
    }
}

//Pause button interrupt, gets called every time an falling edge is detected
void IRAM_ATTR PauseInterrupt(void *param)
{
    //If enough time has passed for the interrupt call to be concidered legimate, send command
    int newPress = FallingEdgeDebouncing(&TrgTickStampPause, &TrgPrevTickStampPause, TrgDebouncingLimit);

    if(newPress == 1)
    {
        SendPause = 1;  //Set flag for sending the command in main loop

        /* TEST:
        //If the timer is in progress, pause timer
        //If the timer is paused, resume timer
        if(TimerState == TimerRunning)
        {
            TimerState = TimerPaused; //TODO: replace with command to livesplit
        }
        else if(TimerState == TimerPaused)
        {
            TimerState = TimerRunning;
        }

        //TODO: for testing, change this to be called when we get new state value from timer
        UpdateLEDState();
        */
    }        
}

//Split button interrupt
void IRAM_ATTR SplitInterrupt(void *param)
{
    //If enough time has passed for the interrupt call to be concidered legimate, send command
    int newPress = FallingEdgeDebouncing(&TrgTickStampSplit, &TrgPrevTickStampSplit, TrgDebouncingLimit);

    if(newPress == 1)
    {
        SendSplit = 1;  //Set flag for sending the command in main loop

        /* TEST:
        switch(TimerState)
        {
            case TimerDefault:     //Timer is in unknown state, set to standby
                TimerState = TimerStandby;
                break;
            case TimerStandby:     //Timer is in standby, start timer
                TimerState = TimerRunning;
                break;
            case TimerRunning:     //Timer is running, stop timer
                TimerState = TimerFinished;
                break;
            case TimerFinished:     //Timer is finished, reset timer
                TimerState = TimerStandby;
                break;
            case TimerPaused:     //Timer is paused, resume timer
                TimerState = TimerRunning;
                break;
            default:
                TimerState = TimerDefault;
                break;
        }

        //TODO: for testing, change this to be called when we get new state value from timer
        UpdateLEDState();
        */
    }
}


//////////////////////////////SETUP//////////////////////////////
void SetupGPIO()
{
    //Setup GPIO input parameters
    ESP_ERROR_CHECK(gpio_config(&PauseBtnConfig));
    ESP_ERROR_CHECK(gpio_config(&SplitBtnConfig));

    //Register GPIO ISR service for interrupts
    //Shared interrupt flag(s) with the fade function
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM|ESP_INTR_FLAG_SHARED);

    //Add interrupt handlers for both buttons
    gpio_isr_handler_add(GPIO_BTN_PAUSE, PauseInterrupt, NULL);
    gpio_isr_handler_add(GPIO_BTN_SPLIT, SplitInterrupt, NULL);

    //Setup PWM output parameters
    ESP_ERROR_CHECK(ledc_channel_config(&SplitLEDChannelConfig));
    ESP_ERROR_CHECK(ledc_timer_config(&SplitLEDPWMConfig));

    //Install fade functionality
    ledc_fade_func_install(ESP_INTR_FLAG_IRAM|ESP_INTR_FLAG_SHARED);
}

void SetupEth()
{
    vTaskDelay(pdMS_TO_TICKS(500));  //Oscilator start up delay

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(tcpip_adapter_set_default_eth_handlers());
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &EthernetEvent, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &GotIPEvent, NULL));

    //MAC config
    eth_mac_config_t macConfig = ETH_MAC_DEFAULT_CONFIG();
    macConfig.smi_mdc_gpio_num = PIN_ETH_MDC;
    macConfig.smi_mdio_gpio_num = PIN_ETH_MDIO;
    esp_eth_mac_t *ethMac = esp_eth_mac_new_esp32(&macConfig);

    //PHY config
    eth_phy_config_t phyConfig = ETH_PHY_DEFAULT_CONFIG();
    phyConfig.phy_addr = 0;     //0: Any address is ok, we have only 1 connection
    esp_eth_phy_t *ethPhy = esp_eth_phy_new_lan8720(&phyConfig);

    //Set the eth physical layer enable pin high
    gpio_pad_select_gpio(PIN_PHY_POWER);
    gpio_set_direction(PIN_PHY_POWER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PHY_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  //Wait 10ms for the enable to take effect? Is this needed?

    //Init ethernet, lan8720 config (RJ45 port)
    esp_eth_config_t ethConfig = ETH_DEFAULT_CONFIG(ethMac, ethPhy);
    esp_eth_handle_t ethHandle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&ethConfig, &ethHandle));
    ESP_ERROR_CHECK(esp_eth_start(ethHandle));
}





//////////////////////////////MAIN//////////////////////////////
void app_main()
{
    SetupGPIO();
    SetupEth();    

    //---MAIN LOOP---
    while(1)
    {
        int NewState = 0;

        if(SendSplit == 1)
        {
            LiveSplitQuery(NetworkSocket, CMD_SPLIT);
            SendSplit = 0;
        }

        if(SendPause == 1)
        {
            LiveSplitQuery(NetworkSocket, CMD_PAUSE);
            SendPause = 0;
        }
        
        //Poll timer state
        //LiveSplitQuery(NetworkSocket, CMD_GET_STATE);

        //Check if got response to timer state poll
        //NewState = LiveSplitState(NetworkSocket, TimerState);

        //If we got a new state value from LiveSplit, call LEDCInterrput to update the split btn led
        if(NewState != TimerState)
        {
            TimerState = NewState;
            UpdateLEDState();
        }

        int ledFadeComplete = CheckLEDFadeProgress(SplitLEDChannelConfig, LED_DUTY_MAX);

        if(ledFadeComplete == 1)
        {
            UpdateLEDState();
        }

        //100ms delay
        vTaskDelay(pdMS_TO_TICKS(100));

        //Test: print connection status
        //printf("Connection status: %d\n", CStatus);
    }
}