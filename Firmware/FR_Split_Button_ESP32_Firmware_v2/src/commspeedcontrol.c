
#include "commspeedcontrol.h"

#include "esp_websocket_client.h"
#include "nvs_flash.h"

#include "freertos/event_groups.h"
#include "esp_log.h"

#define NO_DATA_TIMEOUT_SEC 10

static const char *TAG = "WEBSOCKET";

static TimerHandle_t shutdown_signal_timer;
static SemaphoreHandle_t shutdown_sema;


//Speedcontrol IP/Port
#define SPEEDCTRL_ADDR "https://192.168.0.11:9090"

//Speedcontrol commands
#define SPEEDCTRL_CMD_START "timerStart\r\n"
#define SPEEDCTRL_CMD_PAUSE "timerPause\r\n"
#define SPEEDCTRL_CMD_RESET "timerReset\r\n"
#define SPEEDCTRL_CMD_STOP "timerStop\r\n"

//LiveSplit Server timer status mesasges
#define SPEEDCTRL_MSG_STATE_NOT_RUNNING "NotRunning"
#define SPEEDCTRL_MSG_STATE_RUNNING "Running"
#define SPEEDCTRL_MSG_STATE_ENDED "Ended"
#define SPEEDCTRL_MSG_STATE_PAUSED "Paused"

//Timer control command enumeration
#define TIMER_CMD_GET_STATE 0
#define TIMER_CMD_SPLIT 1
#define TIMER_CMD_PAUSE 2
#define TIMER_CMD_RESET 3
#define TIMER_CMD_STOP 4

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


esp_websocket_client_handle_t WSClient;


static void shutdown_signaler(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "No data received for %d seconds, signaling shutdown", NO_DATA_TIMEOUT_SEC);
    xSemaphoreGive(shutdown_sema);
}


static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_CONNECTED");
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_DISCONNECTED");
        break;
    case WEBSOCKET_EVENT_DATA:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_DATA");
        ESP_LOGI(TAG, "Received opcode=%d", data->op_code);
        if (data->op_code == 0x08 && data->data_len == 2) {
            ESP_LOGW(TAG, "Received closed message with code=%d", 256*data->data_ptr[0] + data->data_ptr[1]);
        } else {
            ESP_LOGW(TAG, "Received=%.*s", data->data_len, (char *)data->data_ptr);
        }
        ESP_LOGW(TAG, "Total payload length=%d, data_len=%d, current payload offset=%d\r\n", data->payload_len, data->data_len, data->payload_offset);

        xTimerReset(shutdown_signal_timer, portMAX_DELAY);
        break;
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGI(TAG, "WEBSOCKET_EVENT_ERROR");
        break;
    }
}


static void websocket_app_start(void)
{
    esp_websocket_client_config_t websocket_cfg = {};

    shutdown_signal_timer = xTimerCreate("Websocket shutdown timer", NO_DATA_TIMEOUT_SEC * 1000 / portTICK_PERIOD_MS,
                                         pdFALSE, NULL, shutdown_signaler);
    shutdown_sema = xSemaphoreCreateBinary();

    websocket_cfg.uri = SPEEDCTRL_ADDR;
    
    ESP_LOGI(TAG, "Connecting to %s...", websocket_cfg.uri);

    WSClient = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_register_events(WSClient, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)WSClient);

    esp_websocket_client_start(WSClient);

    xTimerStart(shutdown_signal_timer, portMAX_DELAY);
    xSemaphoreTake(shutdown_sema, portMAX_DELAY);
}


static void WebsocketClose()
{
    esp_websocket_client_close(WSClient, portMAX_DELAY);
    esp_websocket_client_destroy(WSClient);
}



void SpeedCtrlConnect()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("WEBSOCKET_CLIENT", ESP_LOG_DEBUG);
    esp_log_level_set("TRANSPORT_WS", ESP_LOG_DEBUG);
    esp_log_level_set("TRANS_TCP", ESP_LOG_DEBUG);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    websocket_app_start();
}

int SpeedCtrlState(int currentState)
{
    return 0;
}

void SpeedCtrlCommand(int cmd)
{
    if (esp_websocket_client_is_connected(WSClient))
    {
        esp_websocket_client_send_text(WSClient, SPEEDCTRL_CMD_START, strlen(SPEEDCTRL_CMD_START), portMAX_DELAY);
    }    
}