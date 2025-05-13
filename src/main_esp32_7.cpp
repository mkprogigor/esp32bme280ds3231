#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#include "time.h"
#include <GyverBME280.h>
#include "bme280.h"
#include "Wire.h"
#include "ThingSpeak.h"
#include "mydef.h"

GyverBME280 bme;
WiFiClient  wifi_client;
bme280      bme2;

#define DS3231_I2C_ADDRESS 0x68

#define SCREEN_WIDTH 132 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 4
Adafruit_SH1106 dispOLED(OLED_RESET);

char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASS;
unsigned long my_channel_num = 1059748;
const char write_api_key[] = MYTS_WR_APIKEY;
const char read_api_key[]  = MYTS_WR_APIKEY;

static uint32_t gv_task1_ticks = 0;       // quantity mSec passed in task1 
static bool gv_wifi_fine = false;   // quantity mSec when timers use wifi inet con. 

static float    gv_bme_p = 0;
static float    gv_bme_t = 0;
static float    gv_bme_h = 0;
struct tm       gv_tist;      // time stamp
struct_tph      gv_stru_tph;

TaskHandle_t task1h;
TaskHandle_t task2h;
TaskHandle_t task3h;
TaskHandle_t task4h;

SemaphoreHandle_t mutex_serial;
SemaphoreHandle_t mutex_I2C;

static TimerHandle_t timer_sync_time_rtc;
static TimerHandle_t timer_bmem_send_tsp;

/***************************************************************************************/
boolean gf_wifi_con() {
  WiFi.begin(ssid, pass);
  xSemaphoreTake(mutex_serial, 1000);
  Serial.print("Connecting to WiFi => ");
  xSemaphoreGive(mutex_serial);
  for (u8_t i = 0; i < 16; ++i) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print(". ");  vTaskDelay(1000);
    }
    else {
      xSemaphoreTake(mutex_serial, 1000);
      Serial.print(WiFi.localIP());   Serial.print(" => Conected.\n\n"); 
      xSemaphoreGive(mutex_serial);
      xSemaphoreGive(mutex_serial);
      return true;
    }
  }
  xSemaphoreTake(mutex_serial, 1000);
  Serial.println("\nWiFi didn't connect.\n");
  xSemaphoreGive(mutex_serial);
  return false;
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
//      delay(10);
    }
  }
  else {
    xSemaphoreTake(mutex_serial, portMAX_DELAY);
    Serial.println("No networks found.");
    xSemaphoreGive(mutex_serial);
  }
}

void gf_wifi_status() {
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
    
    byte mac[6];
    WiFi.macAddress(mac);
    for (byte i = 6; i > 0; i--) {
      Serial.print(mac[i-1],HEX); Serial.print(":");
    }
    Serial.print("\n\n");
  }
}

float gc_Pa2mmHg(float pressure) {  // convert Pa to mmHg
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
    gv_tist.tm_sec++;
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
  while (bme.isMeasuring()) vTaskDelay(10);
  gv_stru_tph = bme2.f_read_TPH();
  xSemaphoreGive(mutex_I2C);
  gv_bme_t = ((float)(gv_stru_tph.temp1)) / 100;
  gv_bme_p = ((float)(gv_stru_tph.pres1)) / 100;
  gv_bme_h = ((float)(gv_stru_tph.humi1)) / 1000;

  xSemaphoreTake(mutex_serial, 1000);
  Serial.print("Temperat.,  *C: "); Serial.println(gv_bme_t);
  Serial.print("Humidity,    %: "); Serial.println(gv_bme_h);
  Serial.print("Pressure, mmHg: "); Serial.println(gv_bme_p);
  xSemaphoreGive(mutex_serial);
}

static void gf_timer_sync_time_rtc(TimerHandle_t xTimer) {
  gv_wifi_fine = false;
  xSemaphoreTake(mutex_serial, 1000);
  Serial.print(gv_task1_ticks);  Serial.println(" - Sync time. Timer callBack routine.");
  xSemaphoreGive(mutex_serial);

  if (WiFi.status() != WL_CONNECTED) {
    gf_wifi_con(); // Run only one time to switch ON wifi
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!getLocalTime(&gv_tist)) {
      xSemaphoreTake(mutex_serial, 1000);
      Serial.println("Failed to obtain time.\n");
      xSemaphoreGive(mutex_serial);
      gv_wifi_fine = false;
      xTimerChangePeriod(timer_sync_time_rtc, 60000, 100 );
    }
    else {
      gf_writeDS3231();
      xSemaphoreTake(mutex_serial, 1000);
      Serial.print(&gv_tist);   Serial.print(" => NTP to RTC time update successfully.\n\n");
      xSemaphoreGive(mutex_serial);
      gv_wifi_fine = true;
      xTimerChangePeriod(timer_sync_time_rtc, 3600000, 100 );

      char tv_tist[24];
      tv_tist[0] = (gv_tist.tm_year - 100) / 10 + 48;
      tv_tist[1] = (gv_tist.tm_year - 100) % 10 + 48;
      tv_tist[2] = (gv_tist.tm_mon +1) / 10 + 48;
      tv_tist[3] = (gv_tist.tm_mon +1) % 10 + 48;
      tv_tist[4] = gv_tist.tm_mday / 10 + 48;
      tv_tist[5] = gv_tist.tm_mday % 10 + 48;
      tv_tist[6] = gv_tist.tm_hour / 10 + 48;
      tv_tist[7] = gv_tist.tm_hour % 10 + 48;
      tv_tist[8] = gv_tist.tm_min / 10 + 48;
      tv_tist[9] = gv_tist.tm_min % 10 + 48;
      tv_tist[10] = gv_tist.tm_sec / 10 + 48;
      tv_tist[11] = gv_tist.tm_sec % 10 + 48;
      tv_tist[12] = ' ';
      tv_tist[13] = 'N';
      tv_tist[14] = 'T';
      tv_tist[15] = 'P';
      tv_tist[16] = '2';
      tv_tist[17] = 'R';
      tv_tist[18] = 'T';
      tv_tist[19] = 'C';
      tv_tist[20] = ' ';
      tv_tist[21] = '+';
      tv_tist[22] = '.';
      tv_tist[23] = 0;
      ThingSpeak.setStatus(tv_tist);

    }
  }
  else Serial.println("No connection WiFi.Not sync time.\n");
}

static void gf_timer_bmem_send_tsp(TimerHandle_t xTimer) {
  gv_wifi_fine = false;

  xSemaphoreTake(mutex_serial, 1000);
  Serial.print(gv_task1_ticks);  Serial.println(" - Measure t,p,h. Send to ThinkSpeak. Timer callBack routine.");
  xSemaphoreGive(mutex_serial);

  gf_bmem_tph();

  if (WiFi.status() != WL_CONNECTED) {
    gf_wifi_con(); // Run only one time to switch ON wifi
  }

  if (WiFi.status() == WL_CONNECTED) {
    ThingSpeak.setField(1, gv_bme_t); // set the fields with the values
    ThingSpeak.setField(2, gv_bme_p); // Write to ThingSpeak. There are up to 8 fields in a channel.
    ThingSpeak.setField(3, gv_bme_h);

    int t_ret_code = ThingSpeak.writeFields(my_channel_num, write_api_key);
    xSemaphoreTake(mutex_serial, 1000);
    if (t_ret_code == 200) {
      Serial.println("ThingSpeak ch. update successful.\n");
    }
    else {
      Serial.print("Problem updating channel. HTTP error = "); Serial.println(t_ret_code);
    }
    xSemaphoreGive(mutex_serial);
  }
  else {
    Serial.println("No connection WiFi. Data not send to ThingSpeak.\n");
  }

  gv_wifi_fine = true;
}

void task1_t1ms(void* parameters) { // task1 high priorite, calc xTaskGetTickCount of task1
  while (1) {
    vTaskDelay(1);
    gv_task1_ticks++;
    }
}

void task2_bled(void* parameters) { // blink LED buildin one time in 2 sec
  uint8_t iter_wf = 0;
  while (1) {
    digitalWrite(2, LOW);
    vTaskDelay(1990);
    digitalWrite(2, HIGH);
    vTaskDelay(10);
    if (gv_wifi_fine) {
      if (iter_wf > 10)  {
        if (WiFi.status() == WL_CONNECTED) {
          WiFi.disconnect();
          xSemaphoreTake(mutex_serial, 1000);
          Serial.print(gv_task1_ticks);     Serial.print(" - WiFi disconected.\n\n");
          xSemaphoreGive(mutex_serial);
        }
        iter_wf = 0;
      }
      else iter_wf++;
    }
    else iter_wf = 0;
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

void task4_prni(void* parameters) { // print to serial stats of tasks
  vTaskDelay(35000);
  while (1) {
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
    vTaskDelay(90000);
  }
}

/***************************************************************************************/
void setup() {
  setCpuFrequencyMhz(80);
//  must be First. following frequencies as valid values:
//  240, 160, 80 (For all XTAL types) 40, 20, 10 (For 40MHz XTAL)

Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nStart:");
  Serial.print("CPU Freq, MHz   = ");   Serial.println(getCpuFrequencyMhz());
  Serial.print("XTAL Freq,  MHz = ");   Serial.println(getXtalFrequencyMhz());
  Serial.print("APB Freq, Hz    = ");   Serial.println(getApbFrequency());

  xTaskCreate(task1_t1ms, "task1_t1ms", 1000,  NULL, 6, &task1h);    // Start FreeRTOS
  mutex_serial = xSemaphoreCreateMutex();
  mutex_I2C    = xSemaphoreCreateMutex();

  Wire.begin();
  pinMode(2, OUTPUT);

  dispOLED.begin(SH1106_SWITCHCAPVCC, 0x3C);
  dispOLED.clearDisplay();  dispOLED.setTextColor(WHITE);
  dispOLED.setTextSize(1);  dispOLED.display();

  Serial.print("Check a bme280 => ");
  uint8_t k = bme2.f_check_bme();
  if ( k == 0){
    Serial.println("not found< check cables.");
  }
  else {
    Serial.print("found, code of chip = ");
    Serial.println(k);
  }
  bme2.begin(FOR_MODE, SB_500MS, FIL_x16, OS_x16, OS_x16, OS_x16);

  WiFi.mode(WIFI_STA);
  gf_wifi_con();
  gf_wifi_status();
  vTaskDelay(1000);
  configTime(3600, 3600, "pool.ntp.org");   // init time.h win NTP server, +1 GMT & +1 summer time
  ThingSpeak.begin(wifi_client);  // Initialize ThingSpeak

  xTaskCreate(task2_bled, "task2_bled", 1400, NULL, 5, &task2h);
  xTaskCreate(task3_disp, "task3_disp", 2100, NULL, 4, &task3h);
  xTaskCreate(task4_prni, "task4_prni", 1400, NULL, 2, &task4h);
  timer_bmem_send_tsp = xTimerCreate("bmem_send", 300000, pdTRUE, NULL, gf_timer_bmem_send_tsp);
  xTimerStart(timer_bmem_send_tsp, 0);
  timer_sync_time_rtc = xTimerCreate("sync_time", 3600000, pdTRUE, NULL, gf_timer_sync_time_rtc);
  xTimerStart(timer_sync_time_rtc, 0);
}

void loop() {
  gf_timer_bmem_send_tsp(timer_bmem_send_tsp);
  gf_timer_sync_time_rtc(timer_sync_time_rtc);
  while (1) {
    /* code */
  }
}
/***************************************************************************************/
