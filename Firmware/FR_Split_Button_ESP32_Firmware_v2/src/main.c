
/*
TODO:
Expand the usb slot

TCP/IP communication:
- untested: initialization
- untested: sending commands to livesplit & recieving timer status
- not done: replacing test things in button interrupts to actually sending livesplit commands

Notes:
Is static IP addressing for the buttons needed? We only need to know the
LiveSplit Server's IP.

How is multiple buttons during a race handled? Probably some kind of a custom
command to a custom timer which is done via LiveSplit Core or something similar?
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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

//LiveSplit Server IP/Port
#define LIVESPLIT_IP "192.168.1.1"
#define LIVESPLIT_PORT 16834

//LiveSplit Server command/status messages
#define CMD_SPLIT "startorsplit"
#define CMD_PAUSE "pause"
#define CMD_GET_STATE "getcurrenttimerphase"
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
//True = at max duty
//False = at 0
static bool PWMPhase = false;

//Button falling edge debouncing variables
//Filters out extra interrupt triggers upon pressing a button
//Without filtering, the interrupts can trigger up to 20 times when pressing a button
//Define time in milliseconds at pdMS_TO_TICKS()
unsigned long TrgDebouncingLimit = pdMS_TO_TICKS(250);
unsigned long TrgTickStampSplit = 0;
unsigned long TrgPrevTickStampSplit = 0;
unsigned long TrgTickStampPause = 0;
unsigned long TrgPrevTickStampPause = 0;


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
bool CheckLEDFadeProgress(ledc_channel_config_t cfg, uint32_t maxDuty)
{
    uint32_t ledDuty = ledc_get_duty(cfg.speed_mode, cfg.channel);

    if((ledDuty >= maxDuty) || (ledDuty == 0))
    {
        return true;
    }
    else
    {
        return false;
    }
}

//Called periodicaly whenever the LED duty has reached bottom or top limit, or when the LED is in a static state
void UpdateLEDState()
{
    //Determine next duty value
    uint32_t duty;
   
    PWMPhase = !PWMPhase; //Phase has reached the other end

    if(PWMPhase)
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
            PWMPhase = true;
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
            PWMPhase = false;
            break;
    }
}


//////////////////////////////INTERRUPTS (BUTTONS)//////////////////////////////
//Checks if enough ticks have passed since the last interrupt
bool FallingEdgeDebouncing(unsigned long *tickStamp, unsigned long *prevTickStamp, unsigned long debouncingLimit)
{
    *tickStamp = xTaskGetTickCount();

    if((*tickStamp - debouncingLimit) > *prevTickStamp)
    {
        *prevTickStamp = *tickStamp;
        return true;
    }
    else
    {
        return false;
    }
}

//Pause button interrupt, gets called every time an falling edge is detected
void IRAM_ATTR PauseInterrupt(void *param)
{
    //If enough time has passed for the interrupt call to be concidered legimate, send command
    bool newPress = FallingEdgeDebouncing(&TrgTickStampPause, &TrgPrevTickStampPause, TrgDebouncingLimit);

    if(newPress)
    {
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
    }        
}

//Split button interrupt
void IRAM_ATTR SplitInterrupt(void *param)
{
    //If enough time has passed for the interrupt call to be concidered legimate, send command
    bool newPress = FallingEdgeDebouncing(&TrgTickStampSplit, &TrgPrevTickStampSplit, TrgDebouncingLimit);

    if(newPress)
    {
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
    }
}


//////////////////////////////ETHERNET COMMUNICATION//////////////////////////////
//Event handler for ethernet events, sets MAC-address upon connecting
void EthernetEvent(void *arg, esp_event_base_t eventBase, int32_t eventID, void *eventData)
{
    uint8_t macAddr[6] = {0};
    esp_eth_handle_t ethHandle = *(esp_eth_handle_t *)eventData;

    if(eventID == ETHERNET_EVENT_CONNECTED)
    {
        esp_eth_ioctl(ethHandle, ETH_CMD_G_MAC_ADDR, macAddr);
    }
}


//Sends a command to LiveSplit server
void LiveSplitQuery(int socket, char *command)
{
    //char msg[256] = command; TODO: figure out how to pass the const string into this

    char msg[256] = "startorsplit";
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
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(tcpip_adapter_set_default_eth_handlers());
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &EthernetEvent, NULL));

    eth_mac_config_t macConfig = ETH_MAC_DEFAULT_CONFIG();
    macConfig.smi_mdc_gpio_num = PIN_ETH_MDC;
    macConfig.smi_mdio_gpio_num = PIN_ETH_MDIO;

    eth_phy_config_t phyConfig = ETH_PHY_DEFAULT_CONFIG();
    phyConfig.phy_addr = 0;     //Any address is ok, we have only 1 connection

    //Set the eth physical layer enable pin high
    gpio_pad_select_gpio(PIN_PHY_POWER);
    gpio_set_direction(PIN_PHY_POWER, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PHY_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  //Wait 10ms for the enable to take effect? Is this needed?

    //Initialize ethernet PHY, lan8720 config (RJ45 port)
    esp_eth_mac_t *ethMac = esp_eth_mac_new_esp32(&macConfig);
    esp_eth_phy_t *ethPhy = esp_eth_phy_new_lan8720(&phyConfig);
    esp_eth_config_t ethConfig = ETH_DEFAULT_CONFIG(ethMac, ethPhy);
    esp_eth_handle_t ethHandle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&ethConfig, &ethHandle));
    ESP_ERROR_CHECK(esp_eth_start(ethHandle));

    //Create a socket
    int networkSocket = socket(AF_INET, SOCK_STREAM, 0);
    
    //Specify IP-address for the socket
    struct sockaddr_in serverAddr = 
    {
        .sin_family = AF_INET,
        .sin_port = htons(LIVESPLIT_PORT),
        .sin_addr.s_addr = inet_addr(LIVESPLIT_IP)
    };

    //Connect to the server (LiveSplit)
    int connectionStatus = connect(networkSocket, (struct sockaddr *) &serverAddr, sizeof(serverAddr));

    //Connection failure
    if(connectionStatus == -1)
    {
        printf("Error connecting to LiveSplit!\n");
    }
}





//////////////////////////////MAIN//////////////////////////////
void app_main()
{
    SetupGPIO();
    //SetupEth();    

    //---MAIN LOOP---
    while(1)
    {
        /* COMMENTED OUT FOR LED TESTING

        int NewState;

        //Poll timer state
        LiveSplitQuery(networkSocket, CommandGetState);

        //Check if got response to timer state poll
        NewState = LiveSplitState(networkSocket, TimerState);

        //If we got a new state value from LiveSplit, call LEDCInterrput to update the split btn led
        if(NewState != TimerState)
        {
            TimerState = NewState;
            LEDCInterrupt(NULL);
        }
        */

        bool ledFadeComplete = CheckLEDFadeProgress(SplitLEDChannelConfig, LED_DUTY_MAX);

        if(ledFadeComplete)
        {
            UpdateLEDState();
        }

        //100ms delay
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}