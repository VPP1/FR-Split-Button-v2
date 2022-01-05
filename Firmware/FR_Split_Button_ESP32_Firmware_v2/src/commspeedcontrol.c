
#include "commspeedcontrol.h"

#include "esp_websocket_client.h"
#include "nvs_flash.h"

#include "freertos/event_groups.h"
#include "esp_log.h"

static const char *TAG = "WEBSOCKET";

//Speedcontrol IP/Port
#define SPEEDCTRL_ADDR "ws://192.168.101.11:9091"

// //Speedcontrol commands
// #define SPEEDCTRL_CMD_GET_STATE "timerGetState"
// #define SPEEDCTRL_CMD_START "timerStart"
// #define SPEEDCTRL_CMD_STOP "timerStop"
// #define SPEEDCTRL_CMD_PAUSE "timerPause"
// #define SPEEDCTRL_CMD_RESET "timerReset"

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

//Last response timestamp
static unsigned long LastResponseTimestamp = 0;

//Websocket client
esp_websocket_client_handle_t WSClient;


static void WebsocketEventHandler(void *handlerArgs, esp_event_base_t base, int32_t eventId, void *eventData)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)eventData;
    switch (eventId)
    {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
            break;
        case WEBSOCKET_EVENT_DATA:
            //Check if we got a single byte as response (timer state)
            if (data->data_len == 1)
            {
                //Update response timestamp
                LastResponseTimestamp = xTaskGetTickCount();

                //Numbers start @ 48 in the ASCII table. Subtract 48 to get the number value
                int state = (data->data_ptr[0] - 48);

                //Check if the state is within expected limits
                if ((state >= TIMER_STATE_STANDBY) && (state <= TIMER_STATE_PAUSED))
                {
                    TimerState = state;
                }    
            }

            //Debug
            ESP_LOGW(TAG, "Total payload length=%d, data_len=%d, current payload offset=%d\r\n", data->payload_len, data->data_len, data->payload_offset);
            ESP_LOGW(TAG, "Received=%.*s", data->data_len, (char *)data->data_ptr);

            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
            break;
    }
}


static void WebsocketStart(void)
{
    esp_websocket_client_config_t websocketCfg = {};

    websocketCfg.uri = SPEEDCTRL_ADDR;
    
    ESP_LOGI(TAG, "Connecting to %s...", websocketCfg.uri);

    WSClient = esp_websocket_client_init(&websocketCfg);
    esp_websocket_register_events(WSClient, WEBSOCKET_EVENT_ANY, WebsocketEventHandler, (void *)WSClient);

    esp_websocket_client_start(WSClient);
}


void SpeedCtrlConnect()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("WEBSOCKET_CLIENT", ESP_LOG_DEBUG);
    esp_log_level_set("TRANSPORT_WS", ESP_LOG_DEBUG);
    esp_log_level_set("TRANS_TCP", ESP_LOG_DEBUG);

    ESP_ERROR_CHECK(nvs_flash_init());

    WebsocketStart();
}


//Returns the connection status
int SpeedCtrlConnectionStatus(unsigned long timeout)
{
    if((xTaskGetTickCount() - LastResponseTimestamp) > timeout)
    {
        TimerState = TIMER_STATE_DEFAULT;
        return 0;
    }
    else
    {
        return 1;
    }
}


int SpeedCtrlState()
{
    return TimerState;
}

void SpeedCtrlCommand(int cmd)
{
    if (esp_websocket_client_is_connected(WSClient))
    {
        char cmdChar[1];
        cmdChar[0] = cmd + '0';
        
        esp_websocket_client_send_text(WSClient, cmdChar, sizeof(cmdChar), portMAX_DELAY);
    }    
}