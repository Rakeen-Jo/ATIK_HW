#include "term_print.h"
#include "debug_cmd.h"
#include "HardwareSerial.h"
#include "version.h"
#include "WiFi.h"
#include "time.h"

HardwareSerial ser_debug(0);
HardwareSerial ser_rs232_0(1);
HardwareSerial ser_rs232_1(2);

const char* ssid      = "ATIK";
const char* password  = "atikorea";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

struct tm timeinfo;

void printLocalTime() {
    // struct tm timeinfo;
    if(!getLocalTime(&timeinfo)) {
        ser_debug.println("Failed to obtain time");
        return;
    }
    ser_debug.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void setup() {
  init_terminal_buffers();
  init_debug_history();

  ser_debug.begin(115200);
  ser_rs232_0.begin(115200, SERIAL_8N1, 12, 13);
  ser_rs232_1.begin(115200);

  ser_debug.printf("Connecting to WiFi[%s] ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    ser_debug.print(".");
  }
  ser_debug.println(" CONNECTED");
  
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  term_printf(TPID_DEBUG, "\r\nInitialize complete\r\n");
}

void loop() {
  uart_tx_service();
  uart_debug_service(TPID_DEBUG);
}

void serialEvent() //DEBUG
{
  rxQ_write(TPID_DEBUG, ser_debug.read());
}

void serialEvent1()
{
  rxQ_write(TPID_RS232_0, ser_rs232_0.read());
}

void serialEvent2()
{
  rxQ_write(TPID_RS232_1, ser_rs232_1.read());
}
