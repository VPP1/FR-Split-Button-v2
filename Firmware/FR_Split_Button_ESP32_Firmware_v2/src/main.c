
/*
TODO:
TCP/IP communication:
- untested: initialization
- not done: sending commands to livesplit & recieving timer status

Notes:
Is static IP addressing for the buttons needed? We only need to know the
LiveSplit Server's IP.

How is multiple buttons during a race handled? Probably some kind of a custom
command to a custom timer which is done via LiveSplit Core or something similar?
*/

#include <stdio.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

//For delays, idk if needed
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "olimex_ethernet.h"

#include "tcpip_adapter.h"
#include "esp_eth.h"

//I/O
#define PauseBtnGPIO 14     //Input_pullup (UEXT)
#define SplitBtnGPIO 2      //Input_pullup (UEXT)
#define SplitLEDGPIO 15     //Output (UEXT)
#define PHYPowerPin 12      //Ethernet physical layer enable -output
#define ETHMDCPin 23
#define ETHMDIOPin 18

//LED dynamization parameters, in milliseconds
#define FadeTimeStandby 1000
#define FadeTimeTimerFinished 100
#define FadeTimeTimerPaused 200

//Max led brightness
//Since we are using 5kHz PWM freqeuncy defined in ledc_timer_config_t,
//it means we have 13-bit resolution duty variable. 
//0% - 100% duty => 0 - 8192 duty variable
#define DutyMax 8192

//Main state variable
//0 = standby
//1 = timer running
//2 = timer finished
//3 = timer paused
static int TimerState = 0;

//PWM phase flag
//True = at max duty
//False = at 0
static bool PWMPhase = false;

#pragma region Configurations
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
    .gpio_num   = SplitLEDGPIO,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .hpoint     = 0,
    .timer_sel  = LEDC_TIMER_0
};

//Button I/O config - input pullup
//Pause button
const static gpio_config_t PauseBtnConfig =
{
    .intr_type = GPIO_PIN_INTR_NEGEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL<<PauseBtnGPIO),
    .pull_up_en = GPIO_PULLUP_ENABLE
};

//Split button
const static gpio_config_t SplitBtnConfig =
{
    .intr_type = GPIO_PIN_INTR_NEGEDGE,
    .mode = GPIO_MODE_INPUT,
    .pin_bit_mask = (1ULL<<SplitBtnGPIO),
    .pull_up_en = GPIO_PULLUP_ENABLE
};
#pragma endregion


void app_main()
{
    //---GPIO SETUP---
    //Init fade - need to pass iram and shared flags for the
    //interrupt routine to work at the end of the fade
    ledc_fade_func_install(ESP_INTR_FLAG_IRAM|ESP_INTR_FLAG_SHARED);

    //Register LEDC ISR service for interrupts
    ledc_isr_register(LEDCInterrupt, NULL, ESP_INTR_FLAG_IRAM|ESP_INTR_FLAG_SHARED, NULL);

    //Setup PWM output parameters
    ESP_ERROR_CHECK(ledc_timer_config(&SplitLEDPWMConfig));
    ESP_ERROR_CHECK(ledc_channel_config(&SplitLEDChannelConfig));

    //Register GPIO ISR service for interrupts
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    //Add interrupt handlers for both buttons
    gpio_isr_handler_add(PauseBtnGPIO, PauseInterrupt, NULL);
    gpio_isr_handler_add(SplitBtnGPIO, SplitInterrupt, NULL);

    //Setup GPIO input parameters
    ESP_ERROR_CHECK(gpio_config(&PauseBtnConfig));
    ESP_ERROR_CHECK(gpio_config(&SplitBtnConfig));


    //---ETHERNET SETUP---
    EthernetGPIOConfigRMII();
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(tcpip_adapter_set_default_eth_handlers());
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &EthernetEvent, NULL));

    eth_mac_config_t macConfig = ETH_MAC_DEFAULT_CONFIG();
    macConfig.smi_mdc_gpio_num = ETHMDCPin;
    macConfig.smi_mdio_gpio_num = ETHMDIOPin;

    eth_phy_config_t phyConfig = ETH_PHY_DEFAULT_CONFIG();
    phyConfig.phy_addr = 0;     //Any address is ok, we have only 1 connection

    //Set the eth physical layer enable pin high
    gpio_pad_select_gpio(PHYPowerPin);
    gpio_set_direction(PHYPowerPin, GPIO_MODE_OUTPUT);
    gpio_set_level(PHYPowerPin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));  //Wait 10ms for the enable to take effect? Is this needed?

    //Initialize ethernet PHY, lan8720 config (RJ45 port)
    esp_eth_mac_t *ethMac = esp_eth_mac_new_esp32(&macConfig);
    esp_eth_phy_t *ethPhy = esp_eth_phy_new_lan8720(&phyConfig);
    esp_eth_config_t ethConfig = ETH_DEFAULT_CONFIG(ethMac, ethPhy);
    esp_eth_handle_t ethHandle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&ethConfig, &ethHandle));
    ESP_ERROR_CHECK(esp_eth_start(ethHandle));

    //---MAIN LOOP---
    while(1)
    {
        //TODO: poll timer every second or so
        vTaskDelay(pdMS_TO_TICKS(10)); //Test: 10ms delay
    }
}


//Set LED fading towards the wanted duty cycle
void LEDFade(ledc_channel_config_t led, uint32_t duty, int fadeTime)
{
    ledc_set_fade_with_time(led.speed_mode, led.channel, duty, fadeTime);
    ledc_fade_start(led.speed_mode, led.channel, LEDC_FADE_NO_WAIT);
}


#pragma region Interrupts (buttons, LEDC fade completion)
//Executes automatically once ledc_set_fade_with_time is done
//Also called when we want to update the LED after state change
void IRAM_ATTR LEDCInterrupt(void *param)
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
        duty = DutyMax; //At low phase, start going towards max value
    }
    
    //Set the led fading at a speed defined for each timer state, or set it a static on/off
    switch(TimerState)
    {
        case 0:
            LEDFade(SplitLEDChannelConfig, duty, FadeTimeStandby);
        case 1:
            ledc_set_duty(SplitLEDChannelConfig.speed_mode, SplitLEDChannelConfig.channel, DutyMax);
            PWMPhase = true;
        case 2:
            LEDFade(SplitLEDChannelConfig, duty, FadeTimeTimerFinished);
        case 3:
            LEDFade(SplitLEDChannelConfig, duty, FadeTimeTimerPaused);
        default:
            ledc_set_duty(SplitLEDChannelConfig.speed_mode, SplitLEDChannelConfig.channel, 0);
            PWMPhase = false;
    }
}

void IRAM_ATTR PauseInterrupt(void *param)
{
    //If the timer is in progress, pause timer
    //If the timer is paused, resume timer
    if(TimerState == 1)
    {
        TimerState = 3; //TODO: replace with command to livesplit
    }
    else if(TimerState == 3)
    {
        TimerState = 1;
    }

    //TODO: for testing, change this to be called when we get new state value from timer
    LEDCInterrupt(NULL);
}

void IRAM_ATTR SplitInterrupt(void *param)
{
    switch(TimerState)
    {
        case 0:     //Timer is in standby, start timer
            TimerState = 1;
        case 1:     //Timer is running, stop timer
            TimerState = 2;
        case 2:     //Timer is finished, reset timer
            TimerState = 0;
        case 3:     //Timer is paused, resume timer
            TimerState = 1;
        default:
            TimerState = 0;
    }

    //TODO: for testing, change this to be called when we get new state value from timer
    LEDCInterrupt(NULL);
}
#pragma endregion


#pragma region Ethernet communication
void EthernetGPIOConfigRMII()
{
    // RMII data pins are fixed:
    // TXD0 = GPIO19
    // TXD1 = GPIO22
    // TX_EN = GPIO21
    // RXD0 = GPIO25
    // RXD1 = GPIO26
    // CLK == GPIO0
    phy_rmii_configure_data_interface_pins();
    // MDC is GPIO 23, MDIO is GPIO 18
    phy_rmii_smi_configure_pins(ETHMDCPin, ETHMDIOPin);
}



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
#pragma endregion

