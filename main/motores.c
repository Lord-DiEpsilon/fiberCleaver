// motores.c
#include "motores.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include <string.h>
#include "esp_mac.h"

#include "freertos/event_groups.h"

#define EST_TERMINADO   (1 << 0)
#define CORTE_TERMINADO (1 << 1)

EventGroupHandle_t motor_event_group;


// Declaraciones externas
extern uint8_t calibrado;
extern uint8_t ejCut;
extern uint32_t Init;
extern uint32_t estDist;
extern uint32_t cutDist;
extern char status[16];
extern int32_t pasosContadosCort;
extern int32_t pasosContadosEst;
#define M1_PIN GPIO_NUM_37
#define M2_PIN GPIO_NUM_38

typedef enum {
    MOTOR_ESTIRAMIENTO,
    MOTOR_CORTE
} motor_tipo_t;

typedef struct {
    stepper_motor_t *motor;
    int pasos;
    float mm;
    float mm_total;
    int microsteps;
    uint32_t delay_us;
    bool sentido;
    motor_tipo_t tipo;   // <-- agregamos este campo
} motor_task_params_t;



void habilitar_motor(stepper_motor_t *motor) {
    gpio_set_level(motor->enable_pin, 0); // Activar driver
}

void deshabilitar_motor(stepper_motor_t *motor) {
    gpio_set_level(motor->enable_pin, 1); // Apagar driver
}

void init_stepper_motor(stepper_motor_t *motor) {
    gpio_set_direction(motor->step_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(motor->dir_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(motor->enable_pin, GPIO_MODE_OUTPUT);
    deshabilitar_motor(motor);  // Inicialmente deshabilitado
}

void move_stepper_motor(stepper_motor_t *motor, int pasos, bool forward, uint32_t delay_us) {
    habilitar_motor(motor);

    bool real_dir = forward ^ motor->invert_dir;
    gpio_set_level(motor->dir_pin, real_dir);

    for (int i = 0; i < pasos; i++) {
        gpio_set_level(motor->step_pin, 1);
        ets_delay_us(delay_us);
        gpio_set_level(motor->step_pin, 0);
        ets_delay_us(delay_us);
            
        if (i % 100 == 0) {
            vTaskDelay(1);  // cada 100 pasos, cede CPU
        }
    }
    
    deshabilitar_motor(motor);
}

int calibrar_motor(stepper_motor_t *motor, gpio_num_t endstop_pin, uint32_t delay_us) {
    gpio_set_direction(endstop_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(endstop_pin, GPIO_PULLUP_ONLY);

    habilitar_motor(motor);
    gpio_set_level(motor->dir_pin, motor->invert_dir);
    int pasos = 0;

    while (gpio_get_level(endstop_pin) == 1) {
        gpio_set_level(motor->step_pin, 1);
        ets_delay_us(delay_us);
        gpio_set_level(motor->step_pin, 0);
        ets_delay_us(delay_us);
        pasos++;
    }

    vTaskDelay(pdMS_TO_TICKS(200));

    gpio_set_level(motor->dir_pin, !motor->invert_dir);
    for (int i = 0; i < pasos; i++) {
        gpio_set_level(motor->step_pin, 1);
        ets_delay_us(delay_us);
        gpio_set_level(motor->step_pin, 0);
        ets_delay_us(delay_us);
    }

    deshabilitar_motor(motor);
    return pasos;
}

void realizar_calibracion_doble(stepper_motor_t *motor1, stepper_motor_t *motor2, gpio_num_t endstop_pin, uint32_t delay_us) {
    strcpy(status, "Busy");
    calibrado = 0;

    ESP_LOGI("CALIB", "Iniciando calibración de motor 1...");
    pasosContadosEst = calibrar_motor(motor1, endstop_pin, delay_us);
    ESP_LOGI("CALIB", "Motor 1 calibrado con %ld pasos", pasosContadosEst);

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("CALIB", "Iniciando calibración de motor 2...");
    pasosContadosCort = calibrar_motor(motor2, endstop_pin, delay_us);
    ESP_LOGI("CALIB", "Motor 2 calibrado con %ld pasos", pasosContadosCort);

    calibrado = 1;
    strcpy(status, "Ready");
    ESP_LOGI("CALIB", "Calibración completada");
}


void mover_motor_por_mm(stepper_motor_t *motor, float mm, int pasos_totales_calibrados,
                        float distancia_total_mm, bool direccion_positiva,
                        uint32_t delay_us, bool reset_error) {
    static float error_acumulado = 0.0f;

    if (reset_error) {
        error_acumulado = 0.0f;
    }

    strcpy(status, "Busy");

    if (distancia_total_mm <= 0 || pasos_totales_calibrados <= 0) {
        ESP_LOGE("MOVER", "Parámetros de calibración inválidos");
        strcpy(status, "Ready");
        return;
    }

    float pasos_por_mm = (float)pasos_totales_calibrados / distancia_total_mm;
    float pasos_exactos = mm * pasos_por_mm + error_acumulado;
    int pasos_a_mover = (int)(pasos_exactos);

    error_acumulado = pasos_exactos - pasos_a_mover;

    ESP_LOGI("MOVER", "Moviendo %.2f mm (%.2f pasos acumulados, %d pasos ejecutados)", mm, pasos_exactos, pasos_a_mover);

    move_stepper_motor(motor, pasos_a_mover, direccion_positiva, delay_us);

    strcpy(status, "Ready");
}



void devolver_a_inicio(stepper_motor_t *motor1, float dist1_mm, int pasos1, float mm1_total,
                       stepper_motor_t *motor2, float dist2_mm, int pasos2, float mm2_total,
                       uint32_t delay_us, uint8_t seleccion) {
    static float error_acum_m1 = 0.0f;
    static float error_acum_m2 = 0.0f;

    if (Init == 1) {
        strcpy(status, "Busy");

        // ---- MOTOR 1 ----
        if (seleccion == 1 || seleccion == 3) {
            float pasos_por_mm_1 = (float)pasos1 / mm1_total;
            float pasos_exactos_1 = dist1_mm * pasos_por_mm_1 + error_acum_m1;
            int pasos_a_mover_1 = (int)(pasos_exactos_1);
            error_acum_m1 = pasos_exactos_1 - pasos_a_mover_1;

            configurar_microstepping(M1_PIN, M2_PIN, 8);  // usar misma resolución que en movimiento

            ESP_LOGI("REGRESO", "Regresando motor 1: %.2f mm hacia atrás (%.2f pasos, ejecutando %d)", dist1_mm, pasos_exactos_1, pasos_a_mover_1);
            move_stepper_motor(motor1, pasos_a_mover_1, true, delay_us);

            vTaskDelay(pdMS_TO_TICKS(200));
        }

        // ---- MOTOR 2 ----
        if (seleccion == 2 || seleccion == 3) {
            float pasos_por_mm_2 = (float)pasos2 / mm2_total;
            float pasos_exactos_2 = dist2_mm * pasos_por_mm_2 + error_acum_m2;
            int pasos_a_mover_2 = (int)(pasos_exactos_2);
            error_acum_m2 = pasos_exactos_2 - pasos_a_mover_2;

            configurar_microstepping(M1_PIN, M2_PIN, 16);  // usar misma resolución que en movimiento

            ESP_LOGI("REGRESO", "Regresando motor 2: %.2f mm hacia atrás (%.2f pasos, ejecutando %d)", dist2_mm, pasos_exactos_2, pasos_a_mover_2);
            move_stepper_motor(motor2, pasos_a_mover_2, false, delay_us);
        }

        estDist = 0;
        cutDist = 0;

        strcpy(status, "Ready");
    }
}

void configurar_microstepping(gpio_num_t m0_pin, gpio_num_t m1_pin, int resolucion) {
    gpio_set_direction(m0_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(m1_pin, GPIO_MODE_OUTPUT);

    if (resolucion == 8) {
        gpio_set_level(m0_pin, 0);  // GND
        gpio_set_level(m1_pin, 0);  // GND
    } else if (resolucion == 16) {
        gpio_set_level(m0_pin, 1);  // VCC_IO
        gpio_set_level(m1_pin, 1);  // VCC_IO
    } else if (resolucion == 32) {
        gpio_set_level(m0_pin, 1);  // VCC_IO
        gpio_set_level(m1_pin, 0);  // GND
    } else if (resolucion == 64) {
        gpio_set_level(m0_pin, 0);  // GND
        gpio_set_level(m1_pin, 1);  // VCC_IO
    } else {
        ESP_LOGW("TMC", "Resolución %d no soportada, usando 8", resolucion);
        gpio_set_level(m0_pin, 0);
        gpio_set_level(m1_pin, 0);
        resolucion = 8;
    }

    ESP_LOGI("TMC", "Microstepping configurado a %d pasos", resolucion);
}


// === CALCULAR DELAY US A PARTIR DE VELOCIDAD EN mm/s ===
uint32_t calcular_delay_us(float mm_s, int pasos_totales_calibrados, float mm_totales, int microstepping) {
    float pasos_por_mm = (float)(pasos_totales_calibrados * microstepping) / mm_totales;
    float pasos_por_seg = pasos_por_mm * mm_s;

    if (pasos_por_seg <= 0) return 1000; // valor por defecto

    return (uint32_t)(1e6 / pasos_por_seg);
}

// === RUTINA AVANZADA ===
void ejecutar_rutina_estiramiento_y_corte(
    stepper_motor_t *motor_est, int pasos_est, float mm_est, float mm_estirar,
    stepper_motor_t *motor_corte, int pasos_corte, float mm_corte, float mm_corte_total) {

    strcpy(status, "Busy");

    // === MOTOR DE ESTIRAMIENTO ===
    configurar_microstepping(M1_PIN, M2_PIN, 8);
    // uint32_t delay_est = calcular_delay_us(5.0, pasos_est, mm_est, 8); // velocidad de 5 mm/s
    uint32_t delay_est = 500; // velocidad de 5 mm/s
    mover_motor_por_mm(motor_est, mm_estirar, pasos_est * 8, mm_est, false, delay_est, false);

    if (mm_corte_total > 0.0f) {
        // === MOTOR DE CORTE: AVANCE RÁPIDO ===
        float mm_faltante = 1.0f;
        float mm_avance_rápido = mm_corte_total - mm_faltante;

        configurar_microstepping(M1_PIN, M2_PIN, 16);
        // uint32_t delay_corte_rápido = calcular_delay_us(5.0, pasos_corte, mm_corte, 8);
        uint32_t delay_corte_rápido = 500;
        mover_motor_por_mm(motor_corte, mm_avance_rápido, pasos_corte * 16, mm_corte, true, delay_corte_rápido, false);

        vTaskDelay(pdMS_TO_TICKS(100));

        // === MOTOR DE CORTE: AVANCE PRECISO CON OSCILACIÓN ===
        configurar_microstepping(M1_PIN, M2_PIN, 16);
        float mm_pos = 0.0f;

        while (mm_pos + 0.2f <= mm_faltante) {
            mover_motor_por_mm(motor_corte, 0.2f, pasos_corte * 16, mm_corte, true, 1000, false);
            mm_pos += 0.2f;

            mover_motor_por_mm(motor_corte, 0.1f, pasos_corte * 16, mm_corte, false, 1000, false);
            mm_pos -= 0.1f;

            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    strcpy(status, "Ready");
    ejCut = 0;
    ESP_LOGI("RUTINA", "Rutina especial completada");
}
void regresar_motor_corte(stepper_motor_t *motor_corte, int pasos_corte, float mm_corte, float mm_corte_total) {
    if (mm_corte_total <= 0.0f) return;

    // Usar siempre la misma configuración de microstepping
    configurar_microstepping(M1_PIN, M2_PIN, 16);
    
    // Calcular distancia exacta a regresar (incluyendo las oscilaciones)
    float distancia_total_regreso = mm_corte_total - 0.1f; // Compensar por la oscilación
    
    // Mover con reinicio de error acumulado
    mover_motor_por_mm(motor_corte, distancia_total_regreso, pasos_corte * 16, mm_corte, false, 500, false);

    cutDist = 0;

    vTaskDelay(pdMS_TO_TICKS(200));
}

void motor_task(void *pvParameters) {
    motor_task_params_t *params = (motor_task_params_t *)pvParameters;

    configurar_microstepping(M1_PIN, M2_PIN, params->microsteps);

    mover_motor_por_mm(params->motor,
                       params->mm_total,
                       params->pasos * params->microsteps,
                       params->mm,
                       params->sentido,
                       params->delay_us,
                       false);

    // Señalar que terminó según el tipo
    if (params->tipo == MOTOR_ESTIRAMIENTO) {
        xEventGroupSetBits(motor_event_group, EST_TERMINADO);
    } else {
        xEventGroupSetBits(motor_event_group, CORTE_TERMINADO);
    }

    free(params);
    vTaskDelete(NULL);
}


void ejecutar_rutina_AV(
   stepper_motor_t *motor_est, int pasos_est, float mm_est, float mm_estirar,
    stepper_motor_t *motor_corte, int pasos_corte, float mm_corte, float mm_corte_total) {

    strcpy(status, "Busy");

    // Crear EventGroup
    motor_event_group = xEventGroupCreate();

    // === Parámetros motor de estiramiento ===
    motor_task_params_t *params_est = malloc(sizeof(motor_task_params_t));
    params_est->motor = motor_est;
    params_est->pasos = pasos_est;
    params_est->mm = mm_est;
    params_est->mm_total = mm_estirar;
    params_est->microsteps = 8;
    params_est->delay_us = 500;
    params_est->sentido = false;

    // === Parámetros motor de corte ===
    motor_task_params_t *params_corte = NULL;
    if (mm_corte_total > 0.0f) {
        params_corte = malloc(sizeof(motor_task_params_t));
        params_corte->motor = motor_corte;
        params_corte->pasos = pasos_corte;
        params_corte->mm = mm_corte;
        params_corte->mm_total = mm_corte_total;
        params_corte->microsteps = 16;
        params_corte->delay_us = 500;
        params_corte->sentido = true;
    }

    // Crear tareas
    xTaskCreatePinnedToCore(motor_task, "motor_est", 4096, params_est, 5, NULL, 1);

    if (params_corte) {
        xTaskCreatePinnedToCore(motor_task, "motor_corte", 4096, params_corte, 5, NULL, 1);
    }

    // Esperar a que ambas tareas terminen
    EventBits_t bits_esperados = EST_TERMINADO | (params_corte ? CORTE_TERMINADO : 0);
    xEventGroupWaitBits(motor_event_group,
                        bits_esperados,
                        pdTRUE,    // Limpiar bits al salir
                        pdTRUE,    // Esperar todos los bits
                        portMAX_DELAY);

    // Limpieza
    vEventGroupDelete(motor_event_group);

    strcpy(status, "Ready");
    ejCut = 0;
    ESP_LOGI("RUTINA", "Rutina especial completada");
}

