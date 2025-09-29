// motores.h

// Proyecto de servicio social – Universidad de Guanajuato
// Autores: Alejandro Orozco & Profesor Roberto Rojas Laguna
// Licencia: CC BY-NC 4.0

#ifndef MOTORES_H
#define MOTORES_H

#include "driver/gpio.h"
#include <stdbool.h>

// Estructura de motor paso a paso
typedef struct {
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t enable_pin;
    bool invert_dir;
} stepper_motor_t;

// Declaraciones de funciones
void init_stepper_motor(stepper_motor_t *motor);
void habilitar_motor(stepper_motor_t *motor);
void deshabilitar_motor(stepper_motor_t *motor);
void move_stepper_motor(stepper_motor_t *motor, int pasos, bool forward, uint32_t delay_us);
int calibrar_motor(stepper_motor_t *motor, gpio_num_t endstop_pin, uint32_t delay_us);
void realizar_calibracion_doble(stepper_motor_t *motor1, stepper_motor_t *motor2, gpio_num_t endstop_pin, uint32_t delay_us);
void mover_motor_por_mm(stepper_motor_t *motor, float mm, int pasos_totales_calibrados,
                        float distancia_total_mm, bool direccion_positiva,
                        uint32_t delay_us, bool reset_error);
void devolver_a_inicio(stepper_motor_t *motor1, float dist1_mm, int pasos1, float mm1_total,
                       stepper_motor_t *motor2, float dist2_mm, int pasos2, float mm2_total,
                       uint32_t delay_us, uint8_t seleccion);

// Nuevas funciones agregadas
void configurar_microstepping(gpio_num_t ms1_pin, gpio_num_t ms2_pin, int resolucion);
uint32_t calcular_delay_us(float mm_s, int pasos_totales_calibrados, float mm_totales, int microstepping);
void ejecutar_rutina_estiramiento_y_corte(
    stepper_motor_t *motor_est, int pasos_est, float mm_est, float mm_estirar,
    stepper_motor_t *motor_corte, int pasos_corte, float mm_corte, float mm_corte_total);
void regresar_motor_corte(stepper_motor_t *motor_corte, int pasos_corte, float mm_corte, float mm_corte_total);
void move_both_motors(stepper_motor_t *motor1, int steps1, bool dir1,
                      stepper_motor_t *motor2, int steps2, bool dir2,
                      uint32_t delay_us);
void ejecutar_rutina_AV(
    stepper_motor_t *motor_est, int pasos_est, float mm_est, float mm_estirar,
    stepper_motor_t *motor_corte, int pasos_corte, float mm_corte, float mm_corte_total);

#endif // MOTORES_H