#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#include "time.h"
#include "bme280.h"
#include "Wire.h"
#include "ThingSpeak.h"
#include "mydef.h"

WiFiClient  wifi_client;
bme280      bme2;

#define GD_ENABLE_SLEEP 0 // 1 = enable sleep 15 sec

#define DS3231_I2C_ADDRESS 0x68

#define SCREEN_WIDTH 132 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 4
Adafruit_SH1106 dispOLED(OLED_RESET);

char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASS;
unsigned long my_channel_num = 1059748;
const char write_api_key[] = MYTS_WR_APIKEY;
const char read_api_key[]  = MYTS_RD_APIKEY;

static uint32_t gv_task1_ticks = 0;       // quantity mSec passed in task1 

static float    gv_bme_p = 0;
static float    gv_bme_t = 0;
static float    gv_bme_h = 0;
struct tm       gv_tist;      // time stamp
struct_tph      gv_stru_tph;  // var structure for T, P, H

TaskHandle_t task1h;
TaskHandle_t task2h;
TaskHandle_t task3h;
TaskHandle_t task4h;

SemaphoreHandle_t mutex_serial;
SemaphoreHandle_t mutex_I2C;

// static TimerHandle_t timer_sync_time_rtc;
// static TimerHandle_t timer_bmem_send_tsp;

RTC_DATA_ATTR uint8_t gv_sleep_count = 0;

/***************************************************************************************/
char gf_byte2char(uint8_t lv_byte1){   // translate 0xBA => 'A'
  uint8_t lv_b1 = lv_byte1 & 0x0F;
  if (lv_b1>9) lv_b1 = lv_b1+55;
  else         lv_b1 = lv_b1+48;
  return lv_b1;
}
void gf_prn_byte(uint8_t lv_byte){   // print byte like "FCh "
  Serial.print(gf_byte2char(lv_byte>>4));
  Serial.print(gf_byte2char(lv_byte));
  Serial.print("h ");
}

uint8_t gf_inc_rtc_reboot()  {    //  var rtc_reboot is in DS3231 by addr 0x07
  uint8_t _lv_t;
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
	Wire.write(0x07);
	if (Wire.endTransmission() != 0) return 0;
	if (Wire.requestFrom(DS3231_I2C_ADDRESS, 1) == 1) {
    _lv_t =  Wire.read();
  }
	else return 0;

  _lv_t++;

	Wire.beginTransmission(DS3231_I2C_ADDRESS);
	Wire.write(0x07);
	Wire.write(_lv_t);
	if (Wire.endTransmission() == 0) return _lv_t;
	else return 0;
}

void gf_prm_cpu_info(){
  Serial.println("=====================  Start MCU Info  =====================");
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  Serial.print("Chip Model      = "); Serial.println( chip_info.model);
  Serial.print("Cores           = "); Serial.println( chip_info.cores);
  Serial.print("Revision number = "); Serial.println( chip_info.revision);
  Serial.print("Full rev.number = "); Serial.println( chip_info.full_revision);
  Serial.print("Features, BIN   = "); Serial.println( chip_info.features, BIN);
  Serial.print("CPU Freq, MHz   = ");   Serial.println(getCpuFrequencyMhz());
  Serial.print("XTAL Freq,  MHz = ");   Serial.println(getXtalFrequencyMhz());
  Serial.print("APB Freq, Hz    = ");   Serial.println(getApbFrequency());
  Serial.print("esp_get_idf_version()              = ");  Serial.println(esp_get_idf_version());
  Serial.print("esp_get_free_heap_size()           = ");  Serial.println(esp_get_free_heap_size());
  Serial.print("heap_caps_get_free_size()          = ");  Serial.println(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  Serial.print("heap_caps_get_largest_free_block() = ");  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  size_t spiram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  if (spiram_size) {
    Serial.print("PSRAM Size: "); Serial.println(spiram_size);
  }
  else Serial.println("No PSRAM detected.");
  Serial.println("=====================   End MCU Info   =====================\n");
}

boolean gf_wifi_con() {
  if (WiFi.status() == WL_CONNECTED)  {
    xSemaphoreTake(mutex_serial, 1000);
    Serial.print(WiFi.localIP());
    Serial.print(" => Conected\n\n");
    xSemaphoreGive(mutex_serial);
    return true;
  }
  else  {
    WiFi.begin(ssid, pass);
    xSemaphoreTake(mutex_serial, 1000);
    Serial.print("Connecting to WiFi => ");
    xSemaphoreGive(mutex_serial);
    for (u8_t i = 0; i < 16; ++i) {
      if (WiFi.status() != WL_CONNECTED) {
        xSemaphoreTake(mutex_serial, 1000);
        Serial.print("? ");
        xSemaphoreGive(mutex_serial);
        vTaskDelay(1000);
      }
      else {
        xSemaphoreTake(mutex_serial, 1000);
        Serial.println(WiFi.localIP());
        Serial.println();
        xSemaphoreGive(mutex_serial);
        return true;
      }
    }
    xSemaphoreTake(mutex_serial, 1000);
    Serial.println(" WiFi didn't connect.\n");
    xSemaphoreGive(mutex_serial);
    return false;    /* code */
  }
}

void gf_wifi_scan() {
  xSemaphoreTake(mutex_serial, portMAX_DELAY);
  Serial.println("Scan WiFi networks =>");
  xSemaphoreGive(mutex_serial);
  u8_t n = WiFi.scanNetworks();
  if (n > 0) {
    for (u8_t i = 0; i < n; ++i) {  // Print SSID and RSSI for each network found
      xSemaphoreTake(mutex_serial, portMAX_DELAY);
      Serial.print(i + 1);
      Serial.print(": SSID=");
      Serial.print(WiFi.SSID(i));
      Serial.print(",\tRSSI=(");
      Serial.print(WiFi.RSSI(i));
      Serial.print("),\tEncr.= " );
      Serial.println(WiFi.encryptionType(i));
      xSemaphoreGive(mutex_serial);
    }
  }
  else {
    xSemaphoreTake(mutex_serial, portMAX_DELAY);
    Serial.println("No networks found.");
    xSemaphoreGive(mutex_serial);
  }
}

void gf_wifi_status() {
  Serial.println("===================== WiFi Status Info =====================");
  byte tv_wifist = WiFi.status();
  Serial.print(tv_wifist); Serial.print(" - ");
  switch (tv_wifist)  {
  case WL_CONNECTED:
    Serial.println("WL_CONNECTED");
    break;
    case WL_NO_SHIELD:
    Serial.println("WL_NO_SHIELD");
    break;
    case WL_IDLE_STATUS:
    Serial.println("WL_IDLE_STATUS");
    break;
    case WL_CONNECT_FAILED:
    Serial.println("WL_CONNECT_FAILED");
    break;
    case WL_NO_SSID_AVAIL:
    Serial.println("WL_NO_SSID_AVAIL");
    break;
    case WL_SCAN_COMPLETED:
    Serial.println("WL_SCAN_COMPLETED");
    break;
    case WL_CONNECTION_LOST:
    Serial.println("WL_CONNECTION_LOST");
    break;
    case WL_DISCONNECTED:
    Serial.println("WL_DISCONNECTED");
    break;
    default:
    Serial.println("undefine.");
    break;
  }
  if (tv_wifist == 3)  {
    Serial.print("IP "); Serial.print(WiFi.localIP());
    Serial.print(", MASK "); Serial.print(WiFi.subnetMask());
    Serial.print(", GATE "); Serial.print(WiFi.gatewayIP());
    Serial.print(", DNS  "); Serial.print(WiFi.dnsIP());
    Serial.print(", MAC: ");
    uint8_t mac[6];
    WiFi.macAddress(mac);
    for (uint8_t i = 6; i > 0; i--) {
      Serial.print(mac[i-1],HEX); Serial.print(":");
    }
    Serial.println();
    gf_wifi_scan();
    Serial.println("=================== End WiFi Status Info ===================");
  }
}

float gf_Pa2mmHg(float pressure) {  // convert Pa to mmHg
	return (float)(pressure * 0.00750061683f);
}

byte gf_decToBcd(byte val)  {   // Convert normal decimal numbers to binary coded decimal
  return(((val/10) << 4) | (val%10)); 
}
byte gf_bcdToDec(byte val)  {   // Convert binary coded decimal to normal decimal numbers
  return((val/16*10) + (val%16));
}

static void gf_writeDS3231() {
  if (gv_tist.tm_sec < 58)  {
    gv_tist.tm_sec++;
    // gv_tist.tm_sec++;
  }
  u8_t ds_arr[7];
  ds_arr[0] = gf_decToBcd(gv_tist.tm_sec);
  ds_arr[1] = gf_decToBcd(gv_tist.tm_min);
  ds_arr[2] = gf_decToBcd(gv_tist.tm_hour);
  ds_arr[3] = gf_decToBcd(gv_tist.tm_wday);
  ds_arr[4] = gf_decToBcd(gv_tist.tm_mday);
  ds_arr[5] = gf_decToBcd(gv_tist.tm_mon + 1);
  ds_arr[6] = gf_decToBcd(gv_tist.tm_year - 100);

  xSemaphoreTake(mutex_I2C, portMAX_DELAY);    // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  for (u8_t i = 0; i < 7; i++)  Wire.write(ds_arr[i]);
  Wire.endTransmission();
  xSemaphoreGive(mutex_I2C);
}

static void gf_readDS3231() {
  u8_t ds_arr[7];   // request seven bytes of data from DS3231 starting from register 00h
  xSemaphoreTake(mutex_I2C, portMAX_DELAY);
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  for (u8_t i = 0; i < 7; i++)  ds_arr[i] = Wire.read();
  xSemaphoreGive(mutex_I2C);
  
  gv_tist.tm_sec  = gf_bcdToDec(ds_arr[0] & 0x7f);
  gv_tist.tm_min  = gf_bcdToDec(ds_arr[1]);
  gv_tist.tm_hour = gf_bcdToDec(ds_arr[2] & 0x3f);
  gv_tist.tm_wday = gf_bcdToDec(ds_arr[3]);
  gv_tist.tm_mday = gf_bcdToDec(ds_arr[4]);
  gv_tist.tm_mon  = gf_bcdToDec(ds_arr[5]) - 1;
  gv_tist.tm_year = gf_bcdToDec(ds_arr[6]) + 100;
}

static void gf_bmem_tph() {
  xSemaphoreTake(mutex_I2C, 1000);
  bme2.f_do_1_meas();
  vTaskDelay(200);
  while (bme2.f_bme_is_meas()) vTaskDelay(10);
  gv_stru_tph = bme2.f_read_TPH();
  xSemaphoreGive(mutex_I2C);
  gv_bme_t = ((float)(gv_stru_tph.temp1)) / 100;
  gv_bme_p = gf_Pa2mmHg(((float)(gv_stru_tph.pres1)) / 100);
  gv_bme_h = ((float)(gv_stru_tph.humi1)) / 1000;

  xSemaphoreTake(mutex_serial, 1000);
  Serial.print("Temperat.,  *C: "); Serial.println(gv_bme_t);
  Serial.print("Humidity,    %: "); Serial.println(gv_bme_h);
  Serial.print("Pressure, mmHg: "); Serial.println(gv_bme_p);
  xSemaphoreGive(mutex_serial);
}

static void gf_sync_ntp2rtc() {
  xSemaphoreTake(mutex_serial, 1000);
  Serial.print(gv_task1_ticks);  Serial.print(" -> Sync time => ");
  xSemaphoreGive(mutex_serial);

  if (WiFi.status() != WL_CONNECTED) {
    gf_wifi_con(); // Run only one time to switch ON wifi
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!getLocalTime(&gv_tist)) {
      xSemaphoreTake(mutex_serial, 1000);
      Serial.print("Failed to obtain time.\n\n");
      xSemaphoreGive(mutex_serial);
    }
    else {
      gf_writeDS3231();
      xSemaphoreTake(mutex_serial, 1000);
      Serial.print(&gv_tist);   Serial.print(". NTP to RTC update success.\n\n");
      xSemaphoreGive(mutex_serial);
    }
  }
  else Serial.println("No connection WiFi.Not sync time.\n");
}

static void gf_bmem_send_ts() {
  if (WiFi.status() != WL_CONNECTED) {
    gf_wifi_con(); // Run only one time to switch ON wifi
  }

  if (WiFi.status() == WL_CONNECTED) {
    ThingSpeak.setField(1, gv_bme_t); // set the fields with the values
    ThingSpeak.setField(2, gv_bme_p); // Write to ThingSpeak. There are up to 8 fields in a channel.
    ThingSpeak.setField(3, gv_bme_h);

    // Forming STATUS string for ThingSpeak.com
    char lv_rtc_dim[8], lv_rtc_str[17] = "235959cFFrFsFzFF";
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(0);
    Wire.endTransmission();
    Wire.requestFrom(DS3231_I2C_ADDRESS, 8);
    for (uint8_t i = 0; i < 8; i++)  lv_rtc_dim[i] = Wire.read();
    lv_rtc_dim[2] = lv_rtc_dim[2] & 0x3f;
    lv_rtc_str[0] = gf_byte2char(lv_rtc_dim[2] >> 4);   lv_rtc_str[1] = gf_byte2char(lv_rtc_dim[2]);
    lv_rtc_str[2] = gf_byte2char(lv_rtc_dim[1] >> 4);   lv_rtc_str[3] = gf_byte2char(lv_rtc_dim[1]);
    lv_rtc_str[4] = gf_byte2char(lv_rtc_dim[0] >> 4);   lv_rtc_str[5] = gf_byte2char(lv_rtc_dim[0]);
    lv_rtc_str[6] = 'c';
    lv_rtc_str[7] = gf_byte2char(lv_rtc_dim[7] >> 4);   lv_rtc_str[8] = gf_byte2char(lv_rtc_dim[7]);
    lv_rtc_str[9] = 'r';
    lv_rtc_str[10] = gf_byte2char(esp_reset_reason());
    lv_rtc_str[11] = 's';
    lv_rtc_str[12] = gf_byte2char(esp_sleep_get_wakeup_cause());
    lv_rtc_str[13] = 'z';
    lv_rtc_str[14] = gf_byte2char(gv_sleep_count >> 4);   lv_rtc_str[15] = gf_byte2char(gv_sleep_count);
    //  Serial.print("Status:time + (HEX)(rtc_count + up_reset + up_sleep + up_zzz) = "); 
    Serial.print(gv_task1_ticks);  Serial.print(" -> ");  Serial.println(lv_rtc_str);
    /*  // Function esp_reset_reason() return RESET reason:
    0 = ESP_RST_UNKNOWN       1 = ESP_RST_POWERON       2 = ESP_RST_EXT       3 = ESP_RST_SW
    4 = ESP_RST_PANIC         5 = ESP_RST_INT_WDT       6 = ESP_RST_TASK_WDT  7 = ESP_RST_WDT
    8 = ESP_RST_DEEPSLEEP     9 = ESP_RST_BROWNOUT      10 = ESP_RST_SDIO     
    // Function esp_sleep_get_wakeup_cause() return SLEEP reason:
    0 = ESP_SLEEP_UNKNOWN         1 = ESP_SLEEP_WAKEUP_ALL        2 = ESP_SLEEP_WAKEUP_EXT0
    3 = ESP_SLEEP_WAKEUP_EXT1     4 = ESP_SLEEP_WAKEUP_TIMER      5 = ESP_SLEEP_WAKEUP_TOUCHPAD
    6 = ESP_SLEEP_WAKEUP_ULP      7 = ESP_SLEEP_WAKEUP_GPIO       8 = ESP_SLEEP_WAKEUP_UART
    9 = ESP_SLEEP_WAKEUP_WIFI     10 = ESP_SLEEP_WAKEUP_COCPU     11 = ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG
    12 = ESP_SLEEP_WAKEUP_BT                */
    ThingSpeak.setStatus(lv_rtc_str);

    int t_ret_code = ThingSpeak.writeFields(my_channel_num, write_api_key);
    xSemaphoreTake(mutex_serial, 1000);
    if (t_ret_code == 200) {
      Serial.print(gv_task1_ticks);
      Serial.println(" -> ThingSpeak ch. update successful.\n");
    }
    else {
      Serial.print(gv_task1_ticks);
      Serial.print(" -> Problem updating channel. HTTP error = "); Serial.println(t_ret_code);
    }
    xSemaphoreGive(mutex_serial);
  }
  else {
    Serial.print(gv_task1_ticks);
    Serial.println(" -> No connection WiFi. Data not send to ThingSpeak.\n");
  }
}

static void gf_prn_itasks() { // print to serial stats of tasks
  xSemaphoreTake(mutex_serial, portMAX_DELAY);
  Serial.print("NumberOfTasks = ");                 Serial.println(uxTaskGetNumberOfTasks());
  Serial.print("Task 1ms / xTaskGetTickCount = ");  Serial.print(gv_task1_ticks);  Serial.print(" / ");
  Serial.println(xTaskGetTickCount());
  Serial.print("Task = "); Serial.print(pcTaskGetName(task1h));
  Serial.print(", StackHigh_WM = "); Serial.println(uxTaskGetStackHighWaterMark(task1h));
  Serial.print("Task = "); Serial.print(pcTaskGetName(task2h));
  Serial.print(", StackHigh_WM = "); Serial.println(uxTaskGetStackHighWaterMark(task2h));
  Serial.print("Task = "); Serial.print(pcTaskGetName(task3h));
  Serial.print(", StackHigh_WM = "); Serial.println(uxTaskGetStackHighWaterMark(task3h));
  Serial.print("Task = "); Serial.print(pcTaskGetName(task4h));
  Serial.print(", StackHigh_WM = "); Serial.println(uxTaskGetStackHighWaterMark(task4h));
  Serial.println();
  xSemaphoreGive(mutex_serial);
}

void task1_t1ms(void* parameters) { // task1 high priorite, calc xTaskGetTickCount of task1
  while (1) {
    vTaskDelay(1);
    gv_task1_ticks++;
    }
}

void task2_bled(void* parameters) { // blink LED buildin one time in 2 sec
  while (1) {
    digitalWrite(2, LOW);
    vTaskDelay(1990);
    digitalWrite(2, HIGH);
    vTaskDelay(10);
  }
}

void task3_disp(void* parameters) { // diplay info to OLED display
  while (1) {
    gf_readDS3231();
    xSemaphoreTake(mutex_I2C, portMAX_DELAY);
    dispOLED.clearDisplay();
    dispOLED.setCursor(0, 0);
    dispOLED.print("mS=");     dispOLED.print(gv_task1_ticks);
    dispOLED.print("/");       dispOLED.println(xTaskGetTickCount());
    dispOLED.println();
    dispOLED.println(&gv_tist, "%b %d %a, %T");
    dispOLED.println();
    dispOLED.print("Temp *C: ");    dispOLED.println(gv_bme_t);
    dispOLED.print("Humi  %: ");    dispOLED.println(gv_bme_h);
    dispOLED.print("Pres mmHg: ");  dispOLED.println(gv_bme_p);
    dispOLED.display();
    xSemaphoreGive(mutex_I2C);
    vTaskDelay(1000);
  }
}

void task4_smst(void* parameters) { // sent measurent, sync time
  vTaskDelay(10000);
  uint8_t lv_iter_tph = 0;

  while (1) {
    gf_prn_itasks();
    gf_wifi_con();
    vTaskDelay(1000);

    xSemaphoreTake(mutex_serial, 1000);
    Serial.print(gv_task1_ticks);  Serial.println(" -> Measure t,p,h.");
    xSemaphoreGive(mutex_serial);
    gf_bmem_tph();
  
    xSemaphoreTake(mutex_serial, 1000);
    Serial.print(gv_task1_ticks);  Serial.println(" -> Sent t,p,h. => ThingSpeak.com");
    xSemaphoreGive(mutex_serial);
    gf_bmem_send_ts();

    lv_iter_tph++;
    if (lv_iter_tph > 4) {    // do not update ntp time 4 volte
        lv_iter_tph = 0;
        gf_sync_ntp2rtc();
    }
     
    xSemaphoreTake(mutex_serial, 1000);
    Serial.print(gv_task1_ticks);  Serial.println(" -> WiFi disconnect.\n");
    xSemaphoreGive(mutex_serial);
    WiFi.disconnect();

    // vTaskSuspend(task3h);
    xSemaphoreTake(mutex_I2C, 1000);
    dispOLED.clearDisplay();  dispOLED.display();
    xSemaphoreGive(mutex_I2C);
    gv_bme_t = 0;   gv_bme_p = 0;   gv_bme_h = 0;   // tph => zero.

    if (GD_ENABLE_SLEEP == 1) {
      xSemaphoreTake(mutex_serial, 1000);
      Serial.print(gv_task1_ticks);  Serial.println(" -> go to light sleep mode.");
      xSemaphoreGive(mutex_serial);
      gv_sleep_count++;

      vTaskDelay(1000);
      esp_sleep_enable_timer_wakeup(60000000);  // Go to light sleep 60 sec
      esp_light_sleep_start();                  // possible: esp_deep_sleep_start() OR esp_restart();
      vTaskDelay(1000);
  
      xSemaphoreTake(mutex_serial, 1000);
      Serial.print(gv_task1_ticks);  Serial.print(" -> Wake Up from light sleep mode.\n\n");
      xSemaphoreGive(mutex_serial);
    }

    vTaskDelay(60000);
//  gf_bmem_tph(); Serial.println();
    // vTaskResume(task3h);
  }











}

/***************************************************************************************/
void setup() {
  setCpuFrequencyMhz(80); //  must be First. following frequencies as valid values:
  //  240, 160, 80 (For all XTAL types) 40, 20, 10 (For 40MHz XTAL)

  xTaskCreate(task1_t1ms, "task1_t1ms", 1000, NULL, 6, &task1h);    // Start FreeRTOS
  mutex_serial = xSemaphoreCreateMutex();
  mutex_I2C    = xSemaphoreCreateMutex();

  Serial.begin(115200);
  vTaskDelay(2000);
  Serial.println("\n=====================  Start Setup()   =====================");
  // gf_prm_cpu_info();

  Wire.begin();
  pinMode(2, OUTPUT);
  gf_inc_rtc_reboot();

  dispOLED.begin(SH1106_SWITCHCAPVCC, 0x3C);
  dispOLED.clearDisplay();  dispOLED.setTextColor(WHITE);
  dispOLED.setTextSize(1);  dispOLED.display();

  Serial.print("Check a bme280 => "); // check bme280 and SW reset
  uint8_t k = bme2.f_check_bme();
  vTaskDelay(100);
  if (k == 0) {
    Serial.print("not found, check cables.\n\n");
  }
  else {
    gf_prn_byte(k);
    Serial.print("chip code.\n\n");
  }
  bme2.begin(FOR_MODE, SB_500MS, FIL_x16, OS_x16, OS_x16, OS_x16);

  WiFi.mode(WIFI_STA);
  //  gf_wifi_con();
  //  gf_wifi_status();
  vTaskDelay(1000);
  configTime(3600, 3600, "pool.ntp.org");   // init time.h win NTP server, +1 GMT & +1 summer time
  ThingSpeak.begin(wifi_client);            // Initialize ThingSpeak

  xTaskCreate(task2_bled, "task2_bled", 1000, NULL, 5, &task2h);
  xTaskCreate(task3_disp, "task3_disp", 2100, NULL, 3, &task3h);
  xTaskCreate(task4_smst, "task4_smst", 2400, NULL, 2, &task4h);

  Serial.println("=====================   End   Setup()  =====================\n");
}

void loop() {
  gf_bmem_tph();
  Serial.print("\n\n");
  while (1) {
  }
}
/***************************************************************************************/