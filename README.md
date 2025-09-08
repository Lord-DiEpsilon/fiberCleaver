| Chips Soportados | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- |

# Programa para el control del cortados de fibras

Este programa permite que la placa personalizada sabertooth ESP32 S3, controle dos motores a pasos, correspondiendo uno al corte y otro al estiramiento respectivamente.

El programa está diseñado para utilizar los dos núcleos del ESP32, uno encargado de correr las tareas del BLE, establecer conexiones, enviar-recibir características, etc. Mientras que el otro núcleo se encarga de operar los motores a paso, mediante dos drivers TCM2209 además de detectar finales de carrera para calcular la distancia y como medida de seguridad.

A continuación, se presenta informacion importante del programa, además es posible verificar el manual [aquí.](tutorial/Gatt_Server_Service_Table_Example_Walkthrough.md)

### Hardware Permitido

* La opcion recomendable para implementar el sistema es en la plataforma de desarrollo Sabertooth ESP32 S3, el cual esta a disposicion de IEEE photonics, pero si se requiriera la instalacion en otra plataforma, este programa es compatible con chips: ESP32 S3, C3, C2, C6, H2.

![Plataforma de desarrollo Sabertooth ESP32 S3](misc/PCB.png)

## Archivos

Para lograr actualizar o anadir mas funcionalidades al sistema se debe de editar el archivo principal:

```
\main\gatts_table_creat_demo.c
```

Mientras que para anadir funciones de impplementacion, puede editar el archivo c y de cabeceras:
```
main/motores.c
main/motores.h
```
<!-- ### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://idf.espressif.com/) for full steps to configure and use ESP-IDF to build projects. -->

## Troubleshooting

Para cualquir consulta o sugerencia, puede contactarse mediante el correo institucional a.orozco.ramirez@ugto.mx
