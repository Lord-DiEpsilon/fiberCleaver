// Archivo: cortadora_gatts.c

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_mac.h"
#include <string.h>
#include "esp_bt_device.h"
#include "motores.h"
#include <math.h>   // importante para fabsf()

#define TAG "CORTADORA"
#define DEVICE_NAME "Cortadora"

#define SERVICE_UUID          0xC400
#define CHAR_UUID_CALIBRADO   0xC401
#define CHAR_UUID_EST_DIST    0xC402
#define CHAR_UUID_CUT_DIST    0xC403
#define CHAR_UUID_EJ_CUT      0xC404
#define CHAR_UUID_STATUS      0xC405
#define CHAR_UUID_MOV        0xC407

#define M1_PIN GPIO_NUM_37
#define M2_PIN GPIO_NUM_38
// #define HRS_IDX_NB 20
static esp_gatt_if_t global_gatts_if = 0;

// extern uint8_t calibrado;
// extern char status[16];


enum {
    IDX_SVC,

    IDX_CHAR_CALIBRADO,
    IDX_VAL_CALIBRADO,
    IDX_CFG_CALIBRADO,

    IDX_CHAR_EST_DIST,
    IDX_VAL_EST_DIST,
    IDX_CFG_EST_DIST,

    IDX_CHAR_CUT_DIST,
    IDX_VAL_CUT_DIST,
    IDX_CFG_CUT_DIST,

    IDX_CHAR_EJ_CUT,
    IDX_VAL_EJ_CUT,
    IDX_CFG_EJ_CUT,

    IDX_CHAR_STATUS,
    IDX_VAL_STATUS,
    IDX_CFG_STATUS,

    IDX_CHAR_INIT,       
    IDX_VAL_INIT,
    IDX_CFG_INIT,

    IDX_CHAR_MOV,       
    IDX_VAL_MOV,
    IDX_CFG_MOV,

    HRS_IDX_NB           
};


uint8_t calibrado = 0;
uint8_t mov = 0;          // modo de movimiento
float estDist = 0.0f;       // distancia en mm a estirar
float cutDist = 0.0f;       // distancia en mm a cortar
float estDist_Antes = 0.0f;      // distancia en mm a estirar Anterior
float cutDist_Antes = 0.0f;      // distancia en mm a cortar Anterior

uint8_t ejCut = 0;
uint32_t Init = 0;
int32_t pasosContadosCort = 0;
int32_t pasosContadosEst = 0;

char status[16] = "Ready";

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static uint16_t global_conn_id = 0;
static uint16_t attr_handle_table[HRS_IDX_NB];

stepper_motor_t motor_estirado = {
    .step_pin = GPIO_NUM_2,
    .dir_pin = GPIO_NUM_5,
    .enable_pin = GPIO_NUM_18,
    .invert_dir = true
};

stepper_motor_t motor_corte = {
    .step_pin = GPIO_NUM_3,
    .dir_pin = GPIO_NUM_6,
    .enable_pin = GPIO_NUM_9,
    .invert_dir = false
};

// #include "esp_bt_device.h"

void print_ble_mac() {
    const uint8_t* mac = esp_bt_dev_get_address();
    if (mac == NULL) {
        ESP_LOGE(TAG, "Failed to get BLE MAC address");
        return;
    }
    ESP_LOGI(TAG, "BLE MAC Address: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void task_calibrar(void *arg) {
    configurar_microstepping(M1_PIN, M2_PIN, 8);  // fijamos resolución explícita
    int pasos_micro = calibrar_motor(&motor_estirado, GPIO_NUM_16, 500);
    pasosContadosEst = pasos_micro /8;

    configurar_microstepping(M1_PIN, M2_PIN, 16);
    pasos_micro = calibrar_motor(&motor_corte, GPIO_NUM_16, 500);
    pasosContadosCort = pasos_micro / 16;

    strcpy(status, "Ready");
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
    esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

    vTaskDelete(NULL);
}

void task_ejecutar_corte(void *arg) {
    ESP_LOGI(TAG, "Ejecutando movimiento por ejCut...");

    mover_motor_por_mm(&motor_estirado, (float)estDist, pasosContadosEst*8, 18.0f, false, 500,false);
    vTaskDelay(pdMS_TO_TICKS(200));
    mover_motor_por_mm(&motor_corte, (float)cutDist, pasosContadosCort*16, 2.5f, true, 500,false);
    

    ejCut = 0;
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_EJ_CUT], sizeof(uint8_t), &ejCut);
    ESP_LOGI(TAG, "Movimiento finalizado. ejCut reiniciado a 0");
    strcpy(status, "Ready");
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
    esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

    vTaskDelete(NULL);
}

void task_devolver_a_inicio(void *arg) {
    ESP_LOGI(TAG, "Ejecutando devolver_a_inicio...");

    devolver_a_inicio(&motor_estirado, (float)estDist, pasosContadosEst * 8, 18.0f,
                  &motor_corte, (float)cutDist - 0.1f, pasosContadosCort * 16, 2.5f,
                  500, 1);

    Init = 0;
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_INIT], sizeof(uint32_t), (uint8_t *)&Init);
    ESP_LOGI(TAG, "Init reiniciado a 0");

    strcpy(status, "Ready");
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
    esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

    vTaskDelete(NULL);
}

void task_ejecutar_rutina_avanzada(void *arg) {
    ESP_LOGI(TAG, "Ejecutando rutina avanzada...");

    // Parámetros de calibración (ya definidos tras calibrar)
    int pasos_est = pasosContadosEst;
    int pasos_corte = pasosContadosCort;
    float mm_est = 18.0f;  // según calibración del motor de estiramiento
    float mm_corte = 2.5f; // según calibración del motor de corte

    // Ejecutar rutina avanzada
    ejecutar_rutina_estiramiento_y_corte(
        &motor_estirado, pasos_est, mm_est, (float)estDist,   // estiramiento mm deseado
        &motor_corte, pasos_corte, mm_corte, (float)cutDist   // corte mm deseado
    );
    regresar_motor_corte(&motor_corte, pasosContadosCort, mm_corte, cutDist);

    estDist += 1.5f;

    ejCut = 0;
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_EJ_CUT], sizeof(uint8_t), &ejCut);
    ESP_LOGI(TAG, "Rutina avanzada finalizada. ejCut reiniciado a 0");

    strcpy(status, "Ready");
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
    esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

    vTaskDelete(NULL);
}

void task_selecUbicacionEsti(void *arg) {

    ESP_LOGI(TAG, "Ejecutando selecUbicacion...");

    configurar_microstepping(M1_PIN, M2_PIN, 8);

    if(estDist > estDist_Antes){
        mover_motor_por_mm(&motor_estirado, (float)(estDist-estDist_Antes), pasosContadosEst*8, 18.0f, false, 500,false);
    } else if (estDist < estDist_Antes){
        mover_motor_por_mm(&motor_estirado, (float)(estDist_Antes-estDist), pasosContadosEst*8, 18.0f, true, 500,false);
    }else {
        ESP_LOGI(TAG, "No hay cambio en estDist, no se mueve el motor.");
    }

    estDist_Antes = estDist;

    mov = 0;
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_MOV], sizeof(uint8_t), &mov);
    ESP_LOGI(TAG, "Rutina finalizada. mov reiniciado a 0");

    strcpy(status, "Ready");
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
    esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

    vTaskDelete(NULL);    
}

void task_selecUbicacionCut(void *arg) {

    ESP_LOGI(TAG, "Ejecutando selecUbicacion...");

    configurar_microstepping(M1_PIN, M2_PIN, 16);

    if(cutDist > cutDist_Antes){
        mover_motor_por_mm(&motor_corte, (float)(cutDist-cutDist_Antes), pasosContadosCort*16, 2.5f, true, 500,false);
    } else if (cutDist < cutDist_Antes){
        mover_motor_por_mm(&motor_corte, (float)(cutDist_Antes-cutDist), pasosContadosCort*16, 2.5f, false, 500,false);
    } else {
        ESP_LOGI(TAG, "No hay cambio en cutDist, no se mueve el motor.");
    }

    cutDist_Antes = cutDist;

    mov = 0;
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_MOV], sizeof(uint8_t), &mov);
    ESP_LOGI(TAG, "Rutina finalizada. mov reiniciado a 0");

    strcpy(status, "Ready");
    esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
    esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

    vTaskDelete(NULL);    
}

static void start_advertising(void) {
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = true,  // Esto es crucial para que el nombre aparezca
        .include_txpower = true,
        .min_interval = 0x20,
        .max_interval = 0x40,
        .appearance = 0x0000,  // Appearance genérico
        .manufacturer_len = 0,
        .p_manufacturer_data = NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = sizeof(uint16_t),
        .p_service_uuid = (uint8_t[]){0x00, 0xC4}, // Little-endian
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };

    esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,  // Advertising conectable
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    // Configurar datos de escaneo también (scan response data)
    esp_ble_adv_data_t scan_rsp_data = {
        .set_scan_rsp = true,
        .include_name = true,
        .include_txpower = true,
    };

    esp_ble_gap_config_adv_data(&adv_data);
    esp_ble_gap_config_adv_data(&scan_rsp_data);  // Configurar scan response
    esp_ble_gap_start_advertising(&adv_params);
}

static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] = {
    // Servicio primario
    [IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
      ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){0x00, 0xC4}}},

    // CALIBRADO
    [IDX_CHAR_CALIBRADO] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
      ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
      (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},

    [IDX_VAL_CALIBRADO] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x01, 0xC4},
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint8_t), sizeof(uint8_t), &calibrado}},

    [IDX_CFG_CALIBRADO] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){0x00, 0x00}}},

    // EST_DIST
    [IDX_CHAR_EST_DIST] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
      ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
      (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},

    [IDX_VAL_EST_DIST] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x02, 0xC4},
    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(float), sizeof(float), (uint8_t *)&estDist}},


    [IDX_CFG_EST_DIST] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){0x00, 0x00}}},

    // CUT_DIST
    [IDX_CHAR_CUT_DIST] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
      ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
      (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},

    [IDX_VAL_CUT_DIST] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x03, 0xC4},
    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(float), sizeof(float), (uint8_t *)&cutDist}},

    [IDX_CFG_CUT_DIST] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){0x00, 0x00}}},

    // EJ_CUT
    [IDX_CHAR_EJ_CUT] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
      ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
      (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},

    [IDX_VAL_EJ_CUT] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x04, 0xC4},
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint8_t), sizeof(uint8_t), &ejCut}},

    [IDX_CFG_EJ_CUT] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){0x00, 0x00}}},

    // STATUS
    [IDX_CHAR_STATUS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
      ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
      (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},

    [IDX_VAL_STATUS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x05, 0xC4},
      ESP_GATT_PERM_READ, 16, sizeof("Ready"), (uint8_t *)status}},

    [IDX_CFG_STATUS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){0x00, 0x00}}},

    // INIT
    [IDX_CHAR_INIT] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
      ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
      (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},

    [IDX_VAL_INIT] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x06, 0xC4},
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint32_t), sizeof(uint32_t), (uint8_t *)&Init}},

    [IDX_CFG_INIT] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){0x00, 0x00}}}

    // MOV  
    ,[IDX_CHAR_MOV] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid,
      ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE,
      (uint8_t[]){ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY}}},

    [IDX_VAL_MOV] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t[]){0x07, 0xC4},
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint8_t), sizeof(uint8_t), &mov}},
    
    [IDX_CFG_MOV] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid,
      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(uint16_t), (uint8_t[]){0x00, 0x00}}}
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed");
            } else {
                ESP_LOGI(TAG, "Advertising started successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising stopped");
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "Update connection params status: %d", param->update_conn_params.status);
            break;
        default:
            ESP_LOGI(TAG, "GAP event: %d", event);
            break;
    }
}

static void print_task(void *arg) {
    while (1) {
        ESP_LOGI(TAG, "Calibrado: %i, estDist: %f, cutDist: %f, ejCut: %i, Init: %lu, status: %s", calibrado, estDist, cutDist, ejCut, Init, status);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == attr_handle_table[IDX_VAL_CALIBRADO]) {
            calibrado = param->write.value[0];
            ESP_LOGI(TAG, "Se recibió calibrado: %d", calibrado);

            if (calibrado == 1) {
                strcpy(status, "Busy");
                esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

                xTaskCreate(task_calibrar, "calib_task", 4096, NULL, 5, NULL);
            }

        } else if (param->write.handle == attr_handle_table[IDX_VAL_MOV]) {
            mov = param->write.value[0];
            ESP_LOGI(TAG, "Se recibió mov: %d", mov);

            if (mov == 1) {
                strcpy(status, "Busy");
                esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

                xTaskCreate(task_selecUbicacionEsti, "mov_task", 4096, NULL, 5, NULL);

            } else if (mov == 2) {
                strcpy(status, "Busy");
                esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

                xTaskCreate(task_selecUbicacionCut, "mov_task", 4096, NULL, 5, NULL);
            }
        }
        
        else if (param->write.handle == attr_handle_table[IDX_VAL_EST_DIST]) {
            memcpy(&estDist, param->write.value, sizeof(float));
            ESP_LOGI(TAG, "estDist actualizado a: %.2f mm", estDist);

        } else if (param->write.handle == attr_handle_table[IDX_VAL_CUT_DIST]) {
            memcpy(&cutDist, param->write.value, sizeof(float));
            ESP_LOGI(TAG, "cutDist actualizado a: %.2f mm", cutDist);

        } else if (param->write.handle == attr_handle_table[IDX_VAL_EJ_CUT]) {
            ejCut = param->write.value[0];
            ESP_LOGI(TAG, "Se recibió ejCut: %d", ejCut);
            ESP_LOGI(TAG, "Estado de calibrado: %d", calibrado);

            if (ejCut == 1 && calibrado == 1) {
                strcpy(status, "Busy");
                esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

                xTaskCreate(task_ejecutar_rutina_avanzada, "ejcut_task", 4096, NULL, 5, NULL);
            }

        } else if (param->write.handle == attr_handle_table[IDX_VAL_INIT]) {
            memcpy(&Init, param->write.value, sizeof(uint32_t));
            ESP_LOGI(TAG, "Init actualizado a: %lu", Init);

            if (Init == 1) {
                strcpy(status, "Busy");
                esp_ble_gatts_set_attr_value(attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status);
                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, attr_handle_table[IDX_VAL_STATUS], strlen(status), (uint8_t *)status, false);

                xTaskCreate(task_devolver_a_inicio, "init_task", 4096, NULL, 5, NULL);
            }
        }
        break;


        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "CONNECTED, conn_id = %d", param->connect.conn_id);
            global_conn_id = param->connect.conn_id;
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status == ESP_GATT_OK) {
                memcpy(attr_handle_table, param->add_attr_tab.handles, sizeof(attr_handle_table));
                esp_ble_gatts_start_service(attr_handle_table[IDX_SVC]);
                print_ble_mac();
                xTaskCreate(print_task, "print_task", 2048, NULL, 5, NULL);
                start_advertising();

                init_stepper_motor(&motor_estirado);
                init_stepper_motor(&motor_corte);
            } else {
                ESP_LOGE(TAG, "Failed to create attribute table, error 0x%x", param->add_attr_tab.status);
            }
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "DISCONNECTED");
            start_advertising();
            break;

        default:
            break;
    }

    if (event == ESP_GATTS_REG_EVT) {
        global_gatts_if = gatts_if;
        esp_ble_gap_set_device_name("Cortadora");
        esp_ble_gatts_create_attr_tab(gatt_db, global_gatts_if, HRS_IDX_NB, 0);
    }
}

void app_main(void) {
    esp_err_t ret;
    
    // Inicializar NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Liberar memoria de Bluetooth Classic si no se usa
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Inicializar controlador Bluetooth
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // Inicializar Bluedroid
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Registrar callbacks
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));

    ESP_ERROR_CHECK(esp_ble_gap_set_device_name("Cortadora"));
    
    // Registrar aplicación GATT
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0x55));
    
    // Configurar MTU
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(500));
}