#ifndef _MYCONFIG_H_
#define _MYCONFIG_H_

  // DEBUG SECTION //  
  #define DEBUG_FN_PRINT_DATA

  // SYSTEM CONFIGURATION //
  #define TIME_OUTPUT_DECIMAL_PLACE           7
  #define TIME_COUNTER_PERIOD                 0.1     // uS
  
  #define VOLTAGE_OUTPUT_DECIMAL_PLACE        4
  #define VOLTAGE_CONVERT_MULTIPILER          0.1379
  #define VOLTAGE_CONVERT_OFFSET              1.2794
  #define VOLTAGE(X)                          ( (((X) / 512.0) * (VOLTAGE_CONVERT_MULTIPILER)) + (VOLTAGE_CONVERT_OFFSET) )


  // GNSS SECTION //
  #define GPS_SENTENCE_MAX_LEN  100

  // STM SECTION //  
  #define STM_SENTENCE_MAX_LEN  100

 


  
  // MQTT SECTION //
  #define MQTT_SELECT 1

  #if MQTT_SELECT == 1
    #define MQTT_SERVER           "test.mosquitto.org"
    #define MQTT_PORT             1883
  #else
    #define MQTT_SERVER           "202.133.190.198"
    #define MQTT_PORT             1883
  #endif

  // MQTT TOPICS NAME
  #define MQTT_PUB_TOPIC        "m_field_payload"
  #define MQTT_SUB_DATA_TOPIC   "m_field_data"
  #define MQTT_SUB_CMD_TOPIC    "m_field_command"

  #define MQTT_TOPIC_PUB_NAME       "mfield"
  #define MQTT_TOPIC_PUB_VER        "v1"
  #define MQTT_TOPIC_PUB_DEV_NAME   "kumwell"
  #define MQTT_TOPIC_PUB_DEV_ID     1



  // CONFIG MQTT JSON EXTRA INFO
  #define MQTT_EXTRA_INFO_ADC
  #define MQTT_EXTRA_INFO_TIMER_COUNTER
  #define MQTT_EXTRA_INFO_TIME

  
#endif
