/*
Programa de arduino que se carga en la ESP-WROOM32 para comunicarse con
ROS2. Crea un nodo en el que publica la temperatura actual del extrusor
y si esta imprimiendo o no. También se suscribe para recibir la temperatura 
objetivo y si tiene que imprimir en ese momento o no.
El otro archivo que hace que se comuniquen está en la siguiente dirección:

/home/alvaro/workspace/ros_ur_driver/src/Universal_Robots_ROS2_Driver/ur_bringup/launch/com_arduino.py

-Para que comience la comunicación primero hay que descargarse micro_ros, y como 
pone en los tutoriales lanzar en un terminal:

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0

-Si da problemas de seguridad dar permisos al USB:

sudo chmod 666 /dev/ttyUSB0

-Finalmente lanzar el archivo .py para comunicarse:

python3 /<poner_la_direccion>/com_arduino.py

*/



#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <math.h>

#define TIMER_TIMEOUT_MS 3000 // Tiempo de espera en milisegundos (5 segundos)

//STEPPER Y TERMOPAR///////////////////////////////////////////////////////
#include <PID_v1.h>
#define RELE 33 //como??(nano)
#define tiempoCiclo 1000
#define termistorPin 34
#define termistorNominalRes 100000
#define termistorNominalTemp 25
#define termistorBValue 3950
#define VoltageDividerResistor 1000
#define LED_PIN 2


double Setpoint, Input, Output;                                     // Define Variables 
double Kp=40, Ki=3, Kd=500;                                         // Especifica parametros iniciales

unsigned long respuestaUltimaTemperatura = 0;
unsigned long lastPIDCalculation = 0;
float prevTemperature = -9999.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int DIR = 26;
const int STEP = 25;
const int ENABLE = 27;
const int M0 = 18, M1 = 19, M2 = 20;

double x;
int pos;

float termistorRes = 0.0;
float steinhart;
/////////////////////////////////////////////////////////////////////////////


// Variables de control del temporizador
unsigned long last_msg_time = 0;
bool timer_expired = false;
int imp=0;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//publisher
rcl_publisher_t publisher;
rcl_publisher_t publisher_temperatura; 
rcl_publisher_t publisher_temperatura_cama; 
rcl_publisher_t publisher_imprimir;      
rclc_executor_t executor_pub;
rcl_timer_t timer;
std_msgs__msg__Int32 temperatura; //Temperatura actual de extrusor
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 temperatura_cama; //Temperatura actual de la cama
std_msgs__msg__Int32 imprimir; //1 SI SE ESTA IMPRIMIENDO, 0 SI NO 


//subscriber
rcl_subscription_t subscriber_temperatura;
rcl_subscription_t subscriber_temperatura_cama;
rcl_subscription_t subscriber_imprimir;
rcl_subscription_t subscriber_longitud;
rclc_executor_t executor_sub;
std_msgs__msg__Int32 temperatura_objetivo; //Temperatura a la que se tiene que poner el extrusor
std_msgs__msg__Int32 longitud_extrusion_objetivo; //La longitud de extrusion, para calcular el paso
std_msgs__msg__Int32 imprimir_objetivo; //Si hay que imprimir o no
std_msgs__msg__Int32 temperatura_cama_objetivo; //Temperatura a la que se tiene que poner la cama





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
        delay(1000);
    }
}

////FUNCIONES DE PUBLISHER Y SUBSCRIBER/////////////////////////////////

void temperatura_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    //pasos.data=temperatura_objetivo.data;
    // Reiniciar el temporizador
    last_msg_time = millis();
    timer_expired = false;
    Setpoint=temperatura_objetivo.data;
}

/*void temperatura_cama_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    // Reiniciar el temporizador
    //last_msg_time = millis();
    //timer_expired = false;
    temperatura_cama.data=temperatura_cama_objetivo.data;
}*/

void imprimir_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    // Reiniciar el temporizador
    last_msg_time = millis();
    timer_expired = false;
    imprimir.data=imprimir_objetivo.data;
    imp=imprimir_objetivo.data;
}

/*void longitud_extrusion_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // Realizar acciones con el mensaje de extrusion
}*/

//Funcion para publicar cada segundo, y en caso de dejar de recibir 
//información de temperatura y pasos objetivo, los establece a 0, para
//dejar de imprimir y calentar
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        //RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        RCSOFTCHECK(rcl_publish(&publisher_temperatura, &temperatura, NULL)); // Publicar temperatura
        //RCSOFTCHECK(rcl_publish(&publisher_temperatura_cama, &temperatura_cama, NULL));           
        //RCSOFTCHECK(rcl_publish(&publisher_imprimir, &imprimir, NULL));             

        // Comprobar si el temporizador ha expirado y detener impresión
        if (millis() - last_msg_time >= TIMER_TIMEOUT_MS)
        {
            // Establecer la temperatura y pasos objetivo a 0
            temperatura_objetivo.data = 0;
            imprimir_objetivo.data = 0;
            imprimir.data=0;
            temperatura_cama_objetivo.data=0;
            longitud_extrusion_objetivo.data=0;
            Setpoint=0;
            imp=0;
            digitalWrite(ENABLE,HIGH);//Apago el motor
            // Marcar que el temporizador ha expirado
            timer_expired = true;
        }
    }
}

void leerT(){
  termistorRes = ((float)analogRead (termistorPin)* VoltageDividerResistor)/(4095 - (float)analogRead (termistorPin));
  steinhart = termistorRes / termistorNominalRes;     // (R/Ro)
  steinhart = log(steinhart);                         // ln(R/Ro)
  steinhart /= termistorBValue;                       // 1/B * ln(R/Ro)
  steinhart += 1.0 / (termistorNominalTemp + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                        // Invert  
  steinhart -= 273.15;                                // convert to C

  temperatura.data=round(steinhart);
}

void control() {

    if ((millis() <= (lastPIDCalculation + Output)) || (Output == tiempoCiclo)) {
    // Power on:
    digitalWrite(RELE, HIGH);
  } else {
    // Power off:
    digitalWrite(RELE, LOW);
  }
}

////////////////////////////////////////////////////////////////////////

void setup()
{
    set_microros_transports();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    //STEPPER Y TERMOPAR/////////////////////////////////////////////////////////////////////////
    pinMode(STEP, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(RELE, OUTPUT); 
    pinMode(ENABLE, OUTPUT);
    pinMode(M0, OUTPUT);
    pinMode(M1, OUTPUT); 
    pinMode(M2, OUTPUT);

    //  stepper.setMaxSpeed(1000); //velocidad maxima permitid
    //  stepper.setAcceleration(30);
    //  stepper.setSpeed(1000);// relacion entre aceleracion/desaceleracion

    Setpoint = 0; 
    myPID.SetOutputLimits(0, tiempoCiclo);                           
    myPID.SetSampleTime(tiempoCiclo);
    myPID.SetMode(AUTOMATIC); 
    digitalWrite(DIR,HIGH);
    digitalWrite(ENABLE,HIGH);//Apago el motor
    digitalWrite(M0,HIGH);
    digitalWrite(M1,HIGH);
    digitalWrite(M2,LOW);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    // CREATE PUBLISHER////////////////////////////////////////
    /*RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "micro_ros_arduino_node_publisher"));//nombre del topic ****para lo que se quiera
    */
    RCCHECK(rclc_publisher_init_default(
        &publisher_temperatura, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "temperatura"));//nombre del topic en el que se publica la temperatura del termopar

    /*RCCHECK(rclc_publisher_init_default(
        &publisher_temperatura_cama, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "temperatura_cama"));//nombre del topic en el que se publica la temp de la cama
    RCCHECK(rclc_publisher_init_default(
        &publisher_imprimir, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "imprimir"));//nombre del topic en el que se publica si hay que imprimir o no*/
  ///////////////////////////////////////////////////////////////

    // CREATE SUBSCRIBER //////////////////////////////////////
    RCCHECK(rclc_subscription_init_default(
        &subscriber_temperatura,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "temperatura_objetivo"));//nombre del topic que indica la temp objetivo
    
    /*RCCHECK(rclc_subscription_init_default(
        &subscriber_temperatura_cama,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "temperatura_cama_objetivo"));//nombre del topic que indica la temp objetivo*/
    
    RCCHECK(rclc_subscription_init_default(
        &subscriber_imprimir,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "imprimir_objetivo"));//nombre del topic que indica la temp objetivo

    /*RCCHECK(rclc_subscription_init_default(
        &subscriber_longitud,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "longitud_extrusion_objetivo"));//nombre del topic que indica la temp objetivo*/

  ///////////////////////////////////////////////////////////////

    // create timer,
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

    RCCHECK(rclc_executor_init(&executor_sub, &support.context, 2, &allocator));//el 4 es el numero de ejecutores que maneja
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_temperatura, &temperatura_objetivo, &temperatura_callback, ON_NEW_DATA));
   // RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_temperatura_cama, &temperatura_cama_objetivo, &temperatura_cama_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_imprimir, &imprimir_objetivo, &imprimir_callback, ON_NEW_DATA));
    //RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_longitud, &longitud_extrusion_objetivo, &longitud_extrusion_callback, ON_NEW_DATA));


    msg.data = 0;
    temperatura.data = 0;
    temperatura_cama.data = 0;
    imprimir.data=0;

    temperatura_objetivo.data = 0;
    temperatura_cama_objetivo.data = 0;
    longitud_extrusion_objetivo.data=0;
    imprimir_objetivo.data=0;

    delay(2000);
}

void loop()
{
    //delay(100);
    //RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
    RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, 10));
    RCCHECK(rclc_executor_spin_some(&executor_sub, 10));

////STEPPER Y TERMOPAR////////////////////////////////////////////////////////////////

    if(imp==1){
        digitalWrite(STEP,HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP,LOW);
        delayMicroseconds(500);
    }

    if (millis() - respuestaUltimaTemperatura >= tiempoCiclo) {
        leerT();
        Input = steinhart;
        myPID.Compute();
        lastPIDCalculation = millis();
        respuestaUltimaTemperatura = millis();
    }

    if(imp==0){
        digitalWrite(ENABLE,HIGH);//Apago el motor
    }
    if(imp==1){
        digitalWrite(ENABLE,LOW);//Activo el motor
    }

    control();
}
