
/*
TODO:
Speedcontrol interface:
- Team timer control (multiple buttons)

Button firmware:
- Fix LED getting stuck on dim sometimes on new runs (state 0 -> 1 transition)

Notes:
Add/change to sdkconfig:
CONFIG_ETH_RMII_CLK_OUTPUT=y
CONFIG_ETH_RMII_CLK_OUT_GPIO =17
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

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

#include "commlivesplit.h"
#include "commspeedcontrol.h"

//Interface selection, LiveSplit/Speedcontrol. Uncomment one
//#define INTERFACE_LIVESPLIT
#define INTERFACE_SPEEDCONTROL

//I/O
#define GPIO_BTN_PAUSE 14   //Input_pullup (UEXT)
#define GPIO_BTN_SPLIT 2    //Input_pullup (UEXT)
#define GPIO_LED_SPLIT 15   //Output (UEXT)
#define PIN_PHY_POWER 12    //Ethernet physical layer enable -output
#define PIN_ETH_MDC 23
#define PIN_ETH_MDIO 18

//LED dynamization parameters, in milliseconds
#define TIME_FADE_DEFAULT 100
#define TIME_FADE_STANDBY 4000
#define TIME_FADE_TIMER_FINISHED 500
#define TIME_FADE_TIMER_PAUSED 50

//Max led brightness
//Since we are using 5kHz PWM freqeuncy defined in ledc_timer_config_t,
//it means we have 13-bit resolution duty variable. 
//0% - 100% duty => 0 - 8192 duty variable
#define LED_DUTY_MAX 8192

//Button IP info
#define LOCAL_IP_BTN1 "192.168.0.21"
#define LOCAL_IP_BTN2 "192.168.0.22"
#define SUBNETMASK "255.255.255.0"

//Timer control command enumeration
#define TIMER_CMD_GET_STATE 0
#define TIMER_CMD_START_OR_SPLIT 1
#define TIMER_CMD_STOP 2
#define TIMER_CMD_PAUSE 3
#define TIMER_CMD_RESUME 4
#define TIMER_CMD_RESET 5

//Main state variable
//-1 = default, not connected/error
//0 = not running
//1 = timer running
//2 = timer finished (ended)
//3 = timer paused
#define TIMER_STATE_DEFAULT -1
#define TIMER_STATE_STANDBY 0
#define TIMER_STATE_RUNNING 1
#define TIMER_STATE_FINISHED 2
#define TIMER_STATE_PAUSED 3


static int TimerState = TIMER_STATE_DEFAULT;

//PWM phase flag
//1 = at max duty
//0 = at min duty (0)
static int PWMPhase = 0;

//Button falling edge debouncing variables
//Filters out extra interrupt triggers upon pressing a button
//Without filtering, the interrupts can trigger up to 20 times when pressing a button
//Define time in milliseconds at pdMS_TO_TICKS()
static const unsigned long TrgDebouncingLimit = pdMS_TO_TICKS(250);
static unsigned long TrgTickStampSplit = 0;
static unsigned long TrgPrevTickStampSplit = 0;
static unsigned long TrgTickStampPause = 0;
static unsigned long TrgPrevTickStampPause = 0;

//Button press flags
static int SendSplit = 0;
static int SendPause = 0;



//Connection status monitoring timeout
static const unsigned long ConnectionTimeout = pdMS_TO_TICKS(10000);


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
//Calls the communication setup function of the selected interface
static void Connect()
{
    #if defined(INTERFACE_LIVESPLIT)
        LiveSplitConnect();
    #elif defined(INTERFACE_SPEEDCONTROL)
        SpeedCtrlConnect();
    #endif
}

//Calls the communication function of the selected interface
static void SendCommand(int cmd)
{
    #if defined(INTERFACE_LIVESPLIT)
        LiveSplitCommand(cmd);
    #elif defined(INTERFACE_SPEEDCONTROL)
        SpeedCtrlCommand(cmd);
    #endif
}


//Calls the timer state poll function of the selected interface
static int PollTimerState(int currentState)
{
    int state = currentState;

    #if defined(INTERFACE_LIVESPLIT)
        state = LiveSplitState(currentState);
    #elif defined(INTERFACE_SPEEDCONTROL)
        state = SpeedCtrlState();
    #endif

    //printf("Current state: %d\r\nReturned state: %d\r\n", currentState, state);

    if (currentState != state)
    {
        TimerState = state;
        return 1;
    }
    else
    {
        return 0;
    }
}


//Check if too long has passed since the last response from the server
//0 = connection error, 1 = connection OK
static int ConnectionStatusCheck()
{
    int connectionOK = 0;

    #if defined(INTERFACE_LIVESPLIT)
        connectionOK = LiveSplitConnectionStatus(ConnectionTimeout);
    #elif defined(INTERFACE_SPEEDCONTROL)
        connectionOK = SpeedCtrlConnectionStatus(ConnectionTimeout);
    #endif

    return connectionOK;
}


//Tied to the ethernet event.
void EthernetEvent(void *arg, esp_event_base_t eventBase, int32_t eventID, void *eventData)
{
    uint8_t macAddr[6] = {0};
    esp_eth_handle_t ethHandle = *(esp_eth_handle_t *)eventData;

    tcpip_adapter_ip_info_t ipInfo =
    {
        .ip.addr = ipaddr_addr(LOCAL_IP_BTN1),
        .gw.addr = ipaddr_addr("192.168.0.254"),
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

    Connect();
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
void UpdateLEDState(int timerState)
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
    switch(timerState)
    {
        case TIMER_STATE_DEFAULT:
            ledc_set_duty(SplitLEDChannelConfig.speed_mode, SplitLEDChannelConfig.channel, (LED_DUTY_MAX / 16));
            ledc_update_duty(SplitLEDChannelConfig.speed_mode, SplitLEDChannelConfig.channel);
            PWMPhase = 1;
            break;
        case TIMER_STATE_STANDBY:
            LEDFade(SplitLEDChannelConfig, duty, TIME_FADE_STANDBY);
            break;
        case TIMER_STATE_RUNNING:
            ledc_set_duty(SplitLEDChannelConfig.speed_mode, SplitLEDChannelConfig.channel, LED_DUTY_MAX);
            ledc_update_duty(SplitLEDChannelConfig.speed_mode, SplitLEDChannelConfig.channel);
            PWMPhase = 1;
            break;
        case TIMER_STATE_FINISHED:
            LEDFade(SplitLEDChannelConfig, duty, TIME_FADE_TIMER_FINISHED);
            break;
        case TIMER_STATE_PAUSED:
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
    //If enough time has passed for the interrupt call to be concidered legimate, set send command flag
    if(FallingEdgeDebouncing(&TrgTickStampPause, &TrgPrevTickStampPause, TrgDebouncingLimit) == 1)
    {
        SendPause = 1;  //For some reason the result of falling edge debouncing cannot be cast straight to this
    }
       
}

//Split button interrupt
void IRAM_ATTR SplitInterrupt(void *param)
{
    //If enough time has passed for the interrupt call to be concidered legimate, set send command flag
    if(FallingEdgeDebouncing(&TrgTickStampSplit, &TrgPrevTickStampSplit, TrgDebouncingLimit) == 1)
    {
        SendSplit = 1;
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
        //Check connection status. If connection error is present, update LED
        if(ConnectionStatusCheck() == 0)
        {
            TimerState = TIMER_STATE_DEFAULT;
            UpdateLEDState(TimerState);
        }

        //If button interrupts have written send command flags to 1, send commands to timer
        if(SendSplit == 1)
        {
            switch(TimerState)
            {
                case TIMER_STATE_STANDBY:
                    SendCommand(TIMER_CMD_START_OR_SPLIT);
                    break;
                case TIMER_STATE_RUNNING:
                    #if defined(INTERFACE_LIVESPLIT)
                        SendCommand(TIMER_CMD_START_OR_SPLIT);
                    #elif defined(INTERFACE_SPEEDCONTROL)
                        SendCommand(TIMER_CMD_STOP);
                    #endif
                    break;
                case TIMER_STATE_FINISHED:
                    SendCommand(TIMER_CMD_RESET);
                    break;
                default:
                    break;
            }

            SendSplit = 0;
        }
        
        if(SendPause == 1)
        {
            switch(TimerState)
            {
                case TIMER_STATE_RUNNING:
                    SendCommand(TIMER_CMD_PAUSE);
                    break;
                case TIMER_STATE_PAUSED:
                    #if defined(INTERFACE_LIVESPLIT)
                        SendCommand(TIMER_CMD_RESUME);
                    #elif defined(INTERFACE_SPEEDCONTROL)
                        SendCommand(TIMER_CMD_START_OR_SPLIT);
                    #endif
                    break;
                default:
                    break;
            }

            SendPause = 0;
        }
        

        //Poll timer state
        SendCommand(TIMER_CMD_GET_STATE);

        //If we got a new state value from timer, call LEDCInterrput to update the split btn led
        if(PollTimerState(TimerState) == 1)
        {
            UpdateLEDState(TimerState);
        }

        //If LED fade has reached the end of the fade cycle. If it has, update the LED state
        if(CheckLEDFadeProgress(SplitLEDChannelConfig, LED_DUTY_MAX) == 1)
        {
            UpdateLEDState(TimerState);
        }

        //Update cycle
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}