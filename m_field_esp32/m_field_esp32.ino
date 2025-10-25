//#include "soc/soc.h"
//#include "soc/rtc_cntl_reg.h"
#include <EthernetESP32.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "myconfig.h"

//Time curr_time;


typedef struct {
  //int id;
  uint8_t   t_sec;
  uint8_t   t_min;
  uint8_t   t_hour;
  uint8_t   t_base_sec;

  // ADC value info
  int16_t   max_ns_adc = 111;
  int16_t   max_ew_adc = 9999;

  // Timer Counter info
  uint32_t  max_ns_c = 22222222;
  uint32_t  max_ew_c = 8888888;

  // Time info
  //String    max_ns_t;
  double    max_ns_t_sec;
  //String    max_ew_t;
  double    max_ew_t_sec;
  
  // Voltage info
  float     max_ns_v;
  float     max_ew_v;

  // GNSS Time & Position info
  String    gnss_lat = "777";       // OK
  String    gnss_long = "666";      // OK
  String    gnss_time;
} DataRecord_t;

DataRecord_t m_field_data;





/* 
  * Function prototype
*/
int gps_read();
int stm_read();
void calculate_data();

String build_mqtt_payload();
void publish_data(String mqtt_payload);
void send_data(DataRecord_t rec);
void publish_data(String mqtt_payload);

void gps_uart_send_cmd(char *cmd);
int get_substr(const String str, const char delim, int start_pos, String *ret_str);
void callback(char* topic, byte* payload, unsigned int length);

#ifdef DEBUG_FN_PRINT_DATA
  void print_data();
#endif


/* 
  * Global Variables
*/
HardwareSerial *gps_serial;
HardwareSerial *stm_serial;
HardwareSerial *debug_serial;

const char *mqtt_server = MQTT_SERVER;        // Broker address
IPAddress       ip(192, 168, 1, 111);
EMACDriver      driver(ETH_PHY_LAN8720, 23, 18, 16);
EthernetClient  eth_client;
PubSubClient    mqtt_client(mqtt_server, MQTT_PORT, callback, eth_client);

// End Global Variables


// ====================================================================================================================
//  Core Function
// ====================================================================================================================
/*
  * Setup Function
*/
void setup() {
  serial_setup();
  debug_serial->println("ESP32 started");
  gps_setup();
  ethernet_setup();
  mqtt_setup();
  debug_serial->println("ESP32 Application started\n");
}

/*
  * Loop Function
*/

void loop() {

  // put your main code here, to run repeatedly:
  //DataRecord_t data_rec;
  //int byte_read;

  //String mqtt_pub_str = "This is a test from K**ell";


  // Read GPS first, more important as it is affect timing  
  if (gps_serial->available() != 0) {
    gps_read();
  }


  if (stm_serial->available() != 0) {
    // Mark base time
    m_field_data.t_base_sec = m_field_data.t_sec;
    if(stm_read()) {
      // Interpret and calculate and send data
      calculate_data();
      String mqtt_payload = build_mqtt_payload();
      debug_serial->write(mqtt_payload.c_str());
      debug_serial->println("");

      publish_data(mqtt_payload);
      print_data();
    }
  }

  mqtt_client.loop();
  //delay(5000);
  //debug_serial->println("Testing");
  //mqtt_client.publish(MQTT_PUB_TOPIC, mqtt_pub_str.c_str());
  //send_data(data_rec);
  //mqtt_client.loop();
}
// ====================================================================================================================



// ====================================================================================================================
// ====================================================================================================================

/* Send command to GNSS module function
 * the cmd is exclude start symbol ($) and end symbol (*)
 * this function will put '$' and '*XX<CR><LF>' and send over the uart interface
 */
void gps_uart_send_cmd(char *cmd){
	char chksum = 0;
	char *pcmd = cmd;
  char gps_cmd[100];
	do{
		chksum ^= (*pcmd);
		pcmd++;
	}while(*pcmd != '\0');
  sprintf(gps_cmd,"$%s*%02X\r\n",cmd, chksum);
  debug_serial->print(gps_cmd);
	gps_serial->print(gps_cmd);
  delay(500);
}

/*
  * gps_read Function
  * Read information from GNSS module
*/
int gps_read(){
  
  char gps_byte[GPS_SENTENCE_MAX_LEN];
  int byte_read;
  int index = 0;
  int ret_val = 1;
  
  String gps_sentence;
  String header;  
  String pos_lat;
  String pos_ns;
  String pos_long;
  String pos_ew;
  String time_str;
  String status;
  String mode;
  String chksum;

  byte_read = gps_serial->readBytes(gps_byte, GPS_SENTENCE_MAX_LEN);
  gps_sentence = String(gps_byte);
 
  // TEMP
  //gps_sentence = "$GPGLL,3442.8146,N,13520.1090,E,025411.516,A,A*5D";
  //

  index = get_substr(gps_sentence, ',', index, &header);      // Expect $XXGLL
  index = get_substr(gps_sentence, ',', index, &pos_lat);
  index = get_substr(gps_sentence, ',', index, &pos_ns);
  index = get_substr(gps_sentence, ',', index, &pos_long);
  index = get_substr(gps_sentence, ',', index, &pos_ew);
  index = get_substr(gps_sentence, ',', index, &time_str);
  index = get_substr(gps_sentence, ',', index, &status);
  index = get_substr(gps_sentence, '*', index, &mode);
  
  //index = get_substr(gps_sentence, ',', index, &chksum);
  //debug_serial->printf("Lat : %s,%s Long : %s,%s Time : %s, status : %s, chksum : %s\n", pos_lat.c_str() ,pos_ns.c_str(),  pos_long.c_str(), pos_ew.c_str(), time_str.c_str(), status.c_str(), chksum.c_str());
  //debug_serial->printf("Lat : %s,%s Long : %s,%s Time : %s, status : %s\n", pos_lat.c_str() ,pos_ns.c_str(),  pos_long.c_str(), pos_ew.c_str(), time_str.c_str(), status.c_str());

  // TODO:
  // Check sentence header if it is $XXGLL
  // Check if data valid
  // Check if chksum correct
  // Timeout handle

  m_field_data.gnss_lat = pos_lat + "," + pos_ns;
  m_field_data.gnss_long = pos_long + "," + pos_ew;
  m_field_data.gnss_time = time_str;

  m_field_data.t_hour = time_str.substring(0,2).toInt();
  m_field_data.t_min = time_str.substring(2,4).toInt();
  m_field_data.t_sec = time_str.substring(4,6).toInt();

  return ret_val;
}



/*
  // Format : $STMFIELD,AAAA,BBBB,CCCC,DDDD*XX<CR><LF>
*/
int stm_read(){
  char stm_byte[STM_SENTENCE_MAX_LEN];
  int byte_read;
  int index = 0;
  int ret_val = 1;

  String stm_sentence;
  String header;  
  String max_ns_adc;
  String max_ns_c;
  String max_ew_adc;
  String max_ew_c;
  String chksum;

  byte_read = stm_serial->readBytes(stm_byte, STM_SENTENCE_MAX_LEN);
  stm_sentence = String(stm_byte);

  index = get_substr(stm_sentence, ',', index, &header);        // Expect $STMFIELD
  index = get_substr(stm_sentence, ',', index, &max_ns_adc);    // Expect MAX_NS_ADC
  index = get_substr(stm_sentence, ',', index, &max_ns_c);      // Expect MAX_NS_TIME
  index = get_substr(stm_sentence, ',', index, &max_ew_adc);    // Expect MAX_EW_ADC
  index = get_substr(stm_sentence, ',', index, &max_ew_c);      // Expect MAX_EW_TIME

  m_field_data.max_ns_adc = max_ns_adc.toInt();
  m_field_data.max_ns_c = max_ns_c.toInt();
  m_field_data.max_ew_adc = max_ew_adc.toInt();
  m_field_data.max_ew_c = max_ew_c.toInt();

  return ret_val;
}


/*
  * Prepare the MQTT publish payload
  * Voltage data - 4 decimal points
  * Time data - 7 decimal points
*/
String build_mqtt_payload(){
  JsonDocument json_payload;
  String str;


  // These are mandatory info
  json_payload["MAX_NS_V"] = m_field_data.max_ns_v;       // Max Voltage (4 decimal places)
  //json_payload["MAX_NS_T"] = m_field_data.max_ns_t;       // Time @Max voltage (7 decimal places)
  json_payload["MAX_NS_T"] = String(m_field_data.t_hour) + ":" + String(m_field_data.t_min) + ":" + String(m_field_data.max_ns_t_sec,TIME_OUTPUT_DECIMAL_PLACE);
  //m_field_data.max_ns_t;       // Time @Max voltage (7 decimal places)
  
  json_payload["MAX_EW_V"] = m_field_data.max_ew_v;       // Max Voltage (4 decimal places)
  //json_payload["MAX_EW_T"] = m_field_data.max_ew_t;       // Time @Max voltage (7 decimal places)
  json_payload["MAX_EW_T"] = String(m_field_data.t_hour) + ":" + String(m_field_data.t_min) + ":" + String(m_field_data.max_ew_t_sec,TIME_OUTPUT_DECIMAL_PLACE);
  

  json_payload["LAT"] = m_field_data.gnss_lat;
  json_payload["LONG"] = m_field_data.gnss_long;
  
  // extra info, for validation purpose, can enable/disable in 'myconfig.h'  
  #ifdef MQTT_EXTRA_INFO_ADC
    json_payload["MAX_NS_ADC"] = m_field_data.max_ns_adc;
    json_payload["MAX_EW_ADC"] = m_field_data.max_ew_adc;
  #endif

  #ifdef MQTT_EXTRA_INFO_TIMER_COUNTER
    json_payload["MAX_NS_C"] = m_field_data.max_ns_c;
    json_payload["MAX_EW_C"] = m_field_data.max_ew_c;
  #endif

  #ifdef MQTT_EXTRA_INFO_TIME
    json_payload["GPSTIME"] = m_field_data.gnss_time;
    json_payload["REFTIME_SEC"] = m_field_data.t_base_sec;
  #endif
  
  serializeJson(json_payload, str);
  return str;
}

/*
  * Publish MQTT Payload to server
*/

void publish_data(String mqtt_payload){
  //if() {
    char mqtt_topic[100];

    sprintf(mqtt_topic, "%s/%s/%s%03d", MQTT_TOPIC_PUB_NAME, MQTT_TOPIC_PUB_VER, MQTT_TOPIC_PUB_DEV_NAME,MQTT_TOPIC_PUB_DEV_ID);
    
    debug_serial->printf("Publish %d byte to Topic %s, with data = %s\n", strlen(mqtt_payload.c_str()), mqtt_topic, mqtt_payload.c_str());
    mqtt_client.publish(mqtt_topic, mqtt_payload.c_str());
  //}
}


/*
  Function to extract the substring from the 'start_pos' to the position of the 'delim; minus 1
  return substring to the pointer 'ret_str' and function with the next position of the 'delim'
*/
int get_substr(const String str, const char delim, int start_pos, String *ret_str){
  int end_pos = str.indexOf(delim, start_pos);
  // TODO : Check if the 'delim' cannot be found in 'str'

  *ret_str = str.substring(start_pos, end_pos);
 
  return end_pos + 1;
}

/*
  * This function calculate the voltage from ADC value string ("-512" ... "511"), and round to the defined decimal place (4)
*/
float cal_voltage(int16_t adc_val){
    
  float voltage_adc;
  float voltage_input;
  double multiplier = pow(10.0, VOLTAGE_OUTPUT_DECIMAL_PLACE);
  
  //f = round(VOLTAGE(adc) * multiplier) / multiplier;
  debug_serial->printf("ADC = %d\n", adc_val);
  voltage_adc = ((ADC_VREF_P - ADC_VREF_N) * (adc_val / 512.0)) + ADC_OFFSET;    
  voltage_input = (voltage_adc - 1.0) / (ADC_R2 / (ADC_R1 + ADC_R2));
  // Rounding
  voltage_input= round(voltage_input * multiplier) / multiplier;  
  debug_serial->printf("Calculate ADC val %d to voltage adc = %f, voltage input = %f\n", adc_val, voltage_adc, voltage_input);
  return voltage_input;
}

// t_base in second, t_counter is the counter of timer, each count = TIME_COUNTER_PERIOD (0.1uS)
double cal_time(uint8_t t_base, uint32_t t_counter){
  double sec_offset;
  double t;
  
  double multiplier = pow(10.0, TIME_OUTPUT_DECIMAL_PLACE);
  sec_offset = (t_counter * TIME_COUNTER_PERIOD) / 1000000.0;         // Input is uS,
  t = (round(sec_offset * multiplier) / multiplier ) + t_base;
  debug_serial->printf("Calculate Timer counter %d to time = %f\n", t_counter, t);
  return t;
}

void calculate_data(){
  
  m_field_data.max_ns_v = cal_voltage(m_field_data.max_ns_adc);
  m_field_data.max_ns_t_sec = cal_time(m_field_data.t_base_sec, m_field_data.max_ns_c);

  //m_field_data.max_ns_t = Srting() 
  //sprintf(m_field_data.max_ns_t, "%d:%d:%f", m_field_data.t_hour, m_field_data.t_min, m_field_data.max_ns_t_sec);

  m_field_data.max_ew_v = cal_voltage(m_field_data.max_ew_adc);
  m_field_data.max_ew_t_sec = cal_time(m_field_data.t_base_sec, m_field_data.max_ew_c);
  //sprintf(m_field_data.max_ew_t, "%d:%d:%f", m_field_data.t_hour, m_field_data.t_min, m_field_data.max_ew_t_sec);
}


// ====================================================================================================================





// ================================================================================================================= //
//  Initial & Setup Functions
// ================================================================================================================= //
void serial_setup(){
  // Serial Port mapping
  gps_serial = &Serial1;
  stm_serial = &Serial2;
  debug_serial = &Serial;

  // Serial Port configuration
  gps_serial->begin(38400, SERIAL_8N1, 5, 17);
  stm_serial->begin(115200, SERIAL_8N1, 12, 2);
  debug_serial->begin(115200, SERIAL_8N1, 3, 1);
  delay(2000);
  debug_serial->println("- Serial setup : Done");
}

void gps_setup(){
  debug_serial->print("- GPS Setup : ");
  gps_uart_send_cmd("PERDCFG,NMEAOUT,ALL,0");       // turn off all NMEA output sentences
  gps_uart_send_cmd("PERDAPI,CROUT,DGJPQWXYZ,0");   // turn off all CR output sentences
  gps_uart_send_cmd("PERDCFG,UART1,115200");        // Change GPS uart buadrate
  gps_serial->flush();                              // Ensure all GPS data to be sent
  gps_serial->end();
  delay(1000);

  gps_serial->begin(115200, SERIAL_8N1, 5, 17);     // Chenge ESP32 GPS uart buadrate
  delay(1000);
  gps_uart_send_cmd("PERDSYS,VERSION");             // Check communication
  /*
    TODO : To have code here to read serial port 
    if the version returned corecttly
  */
  gps_uart_send_cmd("PERDCFG,NMEAOUT,GLL,1");       // Enable GLL sentence every second
  debug_serial->println("Done");
}

void ethernet_setup(){
  debug_serial->println("- Setup Ethernet & MQTT");

  Ethernet.init(driver);
  debug_serial->println("-- Initialize Ethernet with DHCP:");
  if (Ethernet.begin() == 0) {
    debug_serial->println("--- Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      debug_serial->println("--- Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      while (true) {
        delay(10000); // do nothing, no point running without Ethernet hardware
        ESP.restart();
      }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      debug_serial->println("--- Ethernet cable is not connected.");
    }
    // try to configure using IP address instead of DHCP:
    Ethernet.begin(ip);
  } else {
    debug_serial->print("--- DHCP assigned IP ");
    debug_serial->println(Ethernet.localIP());
  }
}

void mqtt_setup(){
  const char *mqtt_id = "BJ-Test-1";
  const char *mqtt_user = "";
  const char *mqtt_pass = "";

  mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt_client.setClient(eth_client);
  mqtt_client.setCallback(callback);
  mqtt_client.setBufferSize(512);
  
  while(!mqtt_client.connected()){
    debug_serial->print("Attempting MQTT connection to ");
    debug_serial->println(MQTT_SERVER);

    if (mqtt_client.connect(mqtt_id, mqtt_user, mqtt_pass)){
      debug_serial->println("...connected");
      
      // Once connected, publish an announcement...
      //String data = "Hello from MQTTClient_SSL on " + String(BOARD_NAME);

      //client.publish(TOPIC, data.c_str());

      //Serial.println("Published connection message successfully!");
      //Serial.print("Subcribed to: ");
      //Serial.println(subTopic);
      
      // This is a workaround to address https://github.com/OPEnSLab-OSU/SSLClient/issues/9
      //ethClientSSL.flush();
      // ... and resubscribe
      mqtt_client.subscribe(MQTT_SUB_DATA_TOPIC);
      // for loopback testing
      mqtt_client.subscribe(MQTT_SUB_CMD_TOPIC);
      // This is a workaround to address https://github.com/OPEnSLab-OSU/SSLClient/issues/9
      //ethClientSSL.flush();
    }
    else
    {
      debug_serial->print("...failed, rc=");
      debug_serial->print(mqtt_client.state());
      debug_serial->println(" try again in 5 seconds");

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

























/*






// Send data over MQTT
void send_data(DataRecord_t rec){
  JsonDocument json_payload;
  String mqtt_payload;

  //json_payload["ID"] = rec.id;
  //json_payload["SERIAL"] = rec.serial;
  //json_payload["BASE_TIME"] = "";
  json_payload["NS_MAX_V"] = rec.max_ns_v;
  json_payload["max_ns_t"] = rec.max_ns_t;
  json_payload["MAX_EW_V"] = rec.max_ew_v;
  json_payload["max_ew_t"] = rec.max_ew_t;
  json_payload["LAT"] = rec.pos_lat;
  json_payload["LONG"] = rec.pos_long;
  json_payload["GPSTIME"] = rec.gps_time;
  
  serializeJson(json_payload, mqtt_payload);
  publish_data(mqtt_payload);
}*/












/*
// ================================================================================== //
Key functions and Loop function below
// ================================================================================== //
*/





void callback(char* topic, byte* payload, unsigned int length){
  debug_serial->print("Message arrived [");
  debug_serial->print(topic);
  debug_serial->print("] ");
  
  for (unsigned int i = 0; i < length; i++) 
  {
    debug_serial->print((char)payload[i]);
  }
  
  debug_serial->println();
}


#ifdef DEBUG_FN_PRINT_DATA
  void print_data(){
    DataRecord_t d = m_field_data;
   debug_serial->printf("Max NS ADC Value = %d\n", d.max_ns_adc);
   debug_serial->printf("Max EW ADC Value = %d\n", d.max_ew_adc);
   debug_serial->printf("Max NS Counter Value = %lu\n", d.max_ns_c);
   debug_serial->printf("Max EW Counter Value = %lu\n", d.max_ew_c);
   debug_serial->printf("Max NS Voltage = %f\n", d.max_ns_v);
   debug_serial->printf("Max EW Voltage = %f\n", d.max_ew_v);
   debug_serial->printf("Max NS T_SEC = %lf\n", d.max_ns_t_sec);
   debug_serial->printf("Max EW T_SEC = %lf\n", d.max_ew_t_sec);
   debug_serial->printf("Hour = %d, Min = %d, Sec = %d\n", d.t_hour, d.t_min, d.t_sec);
   debug_serial->printf("Base sec = %d\n", d.t_base_sec);
  }
#endif




