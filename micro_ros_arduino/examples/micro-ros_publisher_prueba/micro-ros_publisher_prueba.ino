#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#define TIMER_TIMEOUT_MS 5000 // Tiempo de espera en milisegundos (5 segundos)

// Variables de control del temporizador
unsigned long last_msg_time = 0;
bool timer_expired = false;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//publisher
rcl_publisher_t publisher;
rcl_publisher_t publisher_temperatura;
rcl_publisher_t publisher_pasos;       
rclc_executor_t executor_pub;
rcl_timer_t timer;
std_msgs__msg__Int32 temperatura;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 pasos;

//subscriber
rcl_subscription_t subscriber;
rcl_subscription_t subscriber_temperatura;
rclc_executor_t executor_sub;
std_msgs__msg__Int32 temperatura_objetivo;
std_msgs__msg__Int32 pasos_objetivo;


#define LED_PIN 13

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            error_loop();              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
        }                              \
    }

// brief loop to indicate error with blinking LED
void error_loop()
{
    while (1)
    {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

////FUNCIONES DE PUBLISHER Y SUBSCRIBER/////////////////////////////////

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    pasos.data=temperatura_objetivo.data;
    // Reiniciar el temporizador
    last_msg_time = millis();
    timer_expired = false;
}

void temperatura_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // Realizar acciones con el mensaje de temperatura
}

//Funcion para publicar cada segundo, y en caso de dejar de recibir 
//información de temperatura y pasos objetivo, los establece a 0, para
//dejar de imprimir y calentar
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        RCSOFTCHECK(rcl_publish(&publisher_temperatura, &temperatura, NULL)); // Publicar temperatura
        RCSOFTCHECK(rcl_publish(&publisher_pasos, &pasos, NULL));             // Publicar pasos
        msg.data++;
        temperatura.data++;
        // Comprobar si el temporizador ha expirado y detener impresión
        if (millis() - last_msg_time >= TIMER_TIMEOUT_MS)
        {
            // Establecer la temperatura y pasos objetivo a 0
            temperatura_objetivo.data = 0;
            pasos_objetivo.data = 0;
            temperatura.data = temperatura.data - 2;//prueba
            // Marcar que el temporizador ha expirado
            timer_expired = true;
        }
    }
}

////////////////////////////////////////////////////////////////////////

void setup()
{
    set_microros_transports();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    delay(2000);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    // CREATE PUBLISHER////////////////////////////////////////
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "micro_ros_arduino_node_publisher"));//nombre del topic ****para lo que se quiera

    RCCHECK(rclc_publisher_init_default(
        &publisher_temperatura, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "temperatura"));//nombre del topic en el que se publica la temperatura del termopar

    RCCHECK(rclc_publisher_init_default(
        &publisher_pasos, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pasos"));//nombre del topic en el que se publican los pasos que se detectan en el extrusor
  ///////////////////////////////////////////////////////////////

    // CREATE SUBSCRIBER //////////////////////////////////////
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "temperatura_objetivo"));//nombre del topic que indica la temp objetivo

  ///////////////////////////////////////////////////////////////

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &temperatura_objetivo, &subscription_callback, ON_NEW_DATA));
    //RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_temperatura, &temperatura_objetivo, &temperatura_callback, ON_NEW_DATA));


    msg.data = 0;
    temperatura.data = 0;
    pasos.data = 0;
    temperatura_objetivo.data = 0;
    pasos_objetivo.data = 0;

}

void loop()
{
    delay(100);
    RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
    RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}
