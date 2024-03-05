#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "slf3s_4000b.h"
#include <math.h>

// ADC moving average calc structure
// using recursive ma algorithm
//
#define     AIN           34
#define			ADC_MA_NUM		10			// ADC moving avg. number

uint16_t    ain;

typedef struct
{
	unsigned char	ma_num;					// moving avg. data  number
	unsigned char	ma_cnt;					// moving avg. new_data write pointer
	
	unsigned short	ma_raw[ADC_MA_NUM];		// keeps raw ADC value
	unsigned long	ma_sum;					// sum
	unsigned short	ma_avg;					// moving avg value

	float   	ma_mV;					// converted ma voltage, unit = V
	// float			ma_mW;					// converted ma current
} ADC_MA_STR;
ADC_MA_STR			adc_ma;

//------------------------------------------------

#define BTN     14

typedef struct
{
	int				kin;					// read key input
	unsigned char	st;						// key status
	unsigned long	prs_st_time;			// key press start time
	unsigned long	rls_st_time;			// key release start time

} KEY_STR;
KEY_STR				key;

//------------------------------------------------
#define   		MODE_flow    		0
#define   		MODE_temp    		1

unsigned char 	disp_mode = MODE_flow;

//------------------------------------------------
#define   		KEY_PRS		   		0		// key press
#define   		KEY_RLS		   		1		// key release

#define   		KEY_DEBOUNCE   		10		// key debounce time, 10msec base, 100msec
#define   		KEY_LONG_PRS   		100		// key long press time, 10msec base, 1 sec

#define   		KEY_ST0_RLS    			0		// key st0 : key released
#define   		KEY_ST1_PRS_CHK			1		// key st1 : key pressed, under debounce checking
#define   		KEY_ST2_PRS    			2		// key st2 : key pressed, debounce checked
#define   		KEY_ST3_RLS_CHK			3		// key st4 : key released, under debounce checking
#define   		KEY_ST4_LONG_PRS		4		// key st3 : key pressed, long key pressed
#define   		KEY_ST5_LONG_RLS_CHK	5		// key st5 : key released, under debounce checking from long press

//------------------------------------------------
// tick(10msec) based time constants
const long 	interval_10ms = 10;      // 10 msec interval 
#define   	TICK_10MS		1
#define   	TICK_50MS    	(TICK_10MS * 5)
#define   	TICK_100MS    	(TICK_10MS * 10)
#define   	TICK_200MS    	(TICK_10MS * 20)
#define   	TICK_300MS    	(TICK_10MS * 30)
#define   	TICK_500MS    	(TICK_10MS * 50)
#define   	TICK_1S    		(TICK_10MS * 100)
#define   	TICK_2S    		(TICK_10MS * 200)

//------------------------------------------------
// variable definitions
//
unsigned long 	prev_time = 0;        
unsigned long 	cur_time = 0;
unsigned long 	tick_10ms = 0;

//------------------------------------------------

#define SLF3S_4000B     0x07030501
#define SLF3S_1300F     0x07030202

#define CONNECTED       0
#define NOT_CONNECTED   1

/* Uncomment the initialize the I2C address , uncomment only one, If you get a totally blank screen try the other*/
// #define oled_addr 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
#define oled_addr 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO

// Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 100000);

const unsigned char ATIK_OLED_LOGO [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x07, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x1F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x3F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x7E, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xF8, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x01, 0xF0, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x03, 0xE0, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF8,
0x00, 0x00, 0x07, 0xC0, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xF8,
0x00, 0x00, 0x0F, 0x80, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xE0,
0x00, 0x00, 0x1F, 0x38, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xC0,
0x00, 0x00, 0x3E, 0x7C, 0x1E, 0x00, 0x07, 0xFF, 0xC7, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0xFF, 0x80,
0x00, 0x00, 0x38, 0xFC, 0x0E, 0x00, 0x0F, 0xFF, 0xCF, 0xFF, 0xFF, 0x3F, 0x00, 0x01, 0xFE, 0x00,
0x00, 0x00, 0x71, 0xFE, 0x0F, 0x00, 0x0F, 0xFF, 0xCF, 0xFF, 0xFE, 0x3F, 0x00, 0x03, 0xFC, 0x00,
0x00, 0x00, 0xE3, 0xCE, 0x0F, 0x00, 0x1F, 0xFF, 0xCF, 0xFF, 0xFE, 0x7E, 0x00, 0x07, 0xF8, 0x00,
0x00, 0x01, 0xC7, 0x8F, 0x0F, 0x00, 0x0E, 0xFF, 0xCF, 0xFF, 0xFC, 0x7E, 0x1F, 0x1F, 0xE0, 0x00,
0x00, 0x03, 0xCF, 0x07, 0x0F, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x00, 0x3F, 0x3F, 0xC0, 0x00,
0x00, 0x07, 0x9E, 0xC7, 0x87, 0x00, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x00, 0x3F, 0x7F, 0x80, 0x00,
0x00, 0x0F, 0x3D, 0xE3, 0x87, 0x80, 0x03, 0xF7, 0xE0, 0x07, 0xE0, 0xFC, 0x7F, 0xFE, 0x00, 0x00,
0x00, 0x0E, 0x7B, 0xE3, 0xC7, 0x80, 0x03, 0xE7, 0xE0, 0x0F, 0xE1, 0xFC, 0x7F, 0xFC, 0x00, 0x00,
0x00, 0x1C, 0xF7, 0xE1, 0xC7, 0x80, 0x07, 0xE7, 0xE0, 0x0F, 0xC1, 0xF8, 0x7F, 0xFC, 0x00, 0x00,
0x00, 0x39, 0xEF, 0xE1, 0xE7, 0x80, 0x0F, 0xC7, 0xE0, 0x0F, 0xC1, 0xF8, 0xFF, 0xFE, 0x00, 0x00,
0x00, 0x7B, 0xCE, 0x07, 0xC7, 0x80, 0x0F, 0xFF, 0xE0, 0x1F, 0xC3, 0xF8, 0xFF, 0xFE, 0x00, 0x00,
0x00, 0xF7, 0x9F, 0xFF, 0xC7, 0xC0, 0x1F, 0xFF, 0xE0, 0x1F, 0x83, 0xF0, 0xFF, 0xFF, 0x00, 0x00,
0x00, 0xEF, 0x1F, 0xFF, 0x83, 0xC0, 0x3F, 0xFF, 0xF0, 0x3F, 0x83, 0xF1, 0xFE, 0x7F, 0x00, 0x00,
0x01, 0xCE, 0x0F, 0xFE, 0x03, 0x80, 0x7F, 0xFF, 0xF0, 0x3F, 0x87, 0xF1, 0xFC, 0x7F, 0x00, 0x00,
0x03, 0xDE, 0x00, 0x00, 0x03, 0x80, 0x7F, 0xFF, 0xF0, 0x3F, 0x07, 0xE3, 0xF8, 0x7F, 0x80, 0x00,
0x07, 0x9F, 0xFF, 0x00, 0x07, 0x80, 0xFE, 0x07, 0xF0, 0x7F, 0x0F, 0xE3, 0xF8, 0x3F, 0x80, 0x00,
0x0F, 0x1F, 0xFF, 0xFF, 0xFF, 0x01, 0xFC, 0x07, 0xF0, 0x7E, 0x0F, 0xE3, 0xF0, 0x3F, 0xC0, 0x00,
0x0E, 0x1F, 0xFF, 0xFF, 0xFF, 0x01, 0xF8, 0x03, 0xF0, 0x7E, 0x0F, 0xC3, 0xF0, 0x1F, 0xC0, 0x00,
0x00, 0x0F, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

uint8_t   slf3s_status = 0;

extern uint32_t  product_num;
extern uint64_t  serial_num;

extern uint16_t  value_flow;
extern uint16_t  value_temp;
extern uint16_t  sig_flag;

uint32_t  last_millis = 0;

uint8_t   res = 0;

void setup() 
{
  Serial.begin(115200);

	pinMode(BTN, INPUT_PULLUP);

  display_init();
  flow_sens_init();
  read_sens_id();

  Serial.println("Complete init!");
  Serial.println();

  display.println("Complete init!");

  display.print("Press btn to start..");

  display.display();
  display.clearDisplay();

  memset(&adc_ma, 0, sizeof(ADC_MA_STR));
	adc_ma.ma_num = ADC_MA_NUM;

  while(digitalRead(BTN))
  ;

  OLED_disp_top();
	delay(100);
	
	OLED_disp_bottom();
	delay(100);
	
	OLED_disp_middle(disp_mode);
	delay(100);

  if (slf3s_status == CONNECTED)
  {
    start_meas_slf3s();

    Serial.println("Start flow meter measurement..");
    Serial.println();
  }

  prev_time = millis();
}

void loop() 
{
  cur_time = millis();
	if (cur_time - prev_time >= interval_10ms) 
	{
		prev_time += interval_10ms;
		tick_10ms++;
		
		// belows will be executed at every 10msec
		key_svc();
    adc_svc();

    if ((tick_10ms % TICK_50MS) == 1)
    {
      OLED_svc();
      
      if (slf3s_status == CONNECTED)
      {
        res = read_slf3s();

        if (res != TRUE)
          Serial.println("CRC error: data");
      }
    }

    if ((tick_10ms % TICK_1S) == 2)
    {
      if (res == TRUE)
        print_debug_slf3s_data();
    }
		
	}
}

void display_init()
{
  delay(250); // wait for the OLED to power up
  display.begin(oled_addr, true); // Address 0x3D default
  display.clearDisplay();   // Clear the buffer.

  display.drawBitmap(0, 0,  ATIK_OLED_LOGO, 128, 64, 1);
  display.display();
  delay(2000);
  display.clearDisplay();   // Clear the buffer.

  // text display settings
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
}

void flow_sens_init()
{ 
  if (init_i2c() != TRUE)
  {
    Serial.println("Error SLF3S init. Stopping proceed..");
    
    display.println("Error SLF3S init.");
    display.println("If you want proceed");
    display.println("without flow meter");
    display.println("Press btn to continue..");

    display.display();

    slf3s_status = NOT_CONNECTED;

    while (digitalRead(BTN))
    ;
  }

  if (slf3s_status == NOT_CONNECTED)
    return;

  Serial.println("SLF3S Complete soft reset");

  display.print("SLF3S");
  for (int i = 0; i < 3; i ++)
  {
    display.print(".");
    display.display();
    delay(500);    
  }
  display.println();
  display.println("Complete soft reset");

  display.display();
  delay(500);
  display.clearDisplay();
}

void read_sens_id()
{
  if (slf3s_status == NOT_CONNECTED)
    return;

  res = read_info_slf3s();   // Read product information (Product number & Serial number)

  display.setCursor(0, 0);

  display.print("Model checking");
  for (int i = 0; i < 3; i ++)
  {
    display.print(".");
    display.display();
    delay(500);    
  }
  display.println();
  display.display();

  if (res == TRUE)
  {
    Serial.println();

    if (product_num == SLF3S_4000B)
    {
      Serial.println("Checked the model : SLF3S-4000B");
      display.println("Model : SLF3S-4000B");
    }
    else if (product_num == SLF3S_1300F)
    {
      Serial.println("Checked the model : SLF3S-1300F");
      display.println("Model : SLF3S-1300F");
    }
    else
    {
      Serial.println("Unable check the model");
      display.println("Unable check the model");
    }

    display.display();

    Serial.print("Product num : 0x");
    Serial.println(product_num, HEX);
    Serial.print("Serial num : 0x");
    Serial.println(serial_num, HEX); 
    Serial.println();

    display.println("");
    display.println("Serial num");
    display.print("0x");
    display.println(serial_num, HEX); 
    display.println();
  }
  else if (res == 0)
  {
    Serial.println("Write error: info");
    display.println("Write error: info");
  }
  else if (res == 2)
  {
    Serial.println("Read error: info");
    display.println("Read error: info");
  }
  else if (res == 3)
  {
    Serial.println("CRC error: info");
    display.println("CRC error: info");
  }
  
  display.display();
}

void print_debug_slf3s_data()
{
  Serial.print("Flow: ");
  Serial.print(value_flow);
  Serial.print(" ml/min");

  Serial.print(" | Temp: ");
  Serial.print(value_temp);
  Serial.print(" deg C");

  Serial.print(" | Signal: 0x");
  Serial.print(sig_flag, HEX);
  Serial.print("\r\n");
}

void adc_svc()
{
	ain = analogRead(AIN);
	adc_ma_svc(ain);			// save raw data & calc. moving avg
	adc_calc_V_mW();			// calc. vtg and mW
	
	adc_ma_chk();
}

// check ADC ma calculation is broken or not
void adc_ma_chk()
{
	int				i;
	unsigned long	sum;	
	
	if ((tick_10ms % TICK_1S) != 1)
		return;

	sum = 0;
	for (i = 0; i < ADC_MA_NUM; i++)
	{
		sum += adc_ma.ma_raw[i];
	}
	
	if (sum != adc_ma.ma_sum)
	{
		Serial.println(" ");
		Serial.print(" - ADC moving avg mismatch found, invalid sum=");
		Serial.print(adc_ma.ma_sum);
		Serial.print(" valid sum=");
		Serial.print(sum);
		Serial.println(" ... corrected");
		Serial.println(" ");
		
		adc_ma.ma_sum = sum;
		adc_ma.ma_avg = (adc_ma.ma_sum + adc_ma.ma_num/2) / adc_ma.ma_num;	// nearest integer avg.
	}
}


void adc_ma_svc(int new_ain)
{
	unsigned char	old_pos;

	// 1. find delete item position
	//    we use fixed ma_buf, so delete position = write position
	old_pos = adc_ma.ma_cnt;
		
	// 2. calc recursive sum & avg
	adc_ma.ma_sum += new_ain;
	adc_ma.ma_sum -= adc_ma.ma_raw[old_pos];
	adc_ma.ma_avg = (adc_ma.ma_sum + adc_ma.ma_num/2) / adc_ma.ma_num;	// nearest integer avg.
	
	// 3. save new value
	adc_ma.ma_raw[adc_ma.ma_cnt] = new_ain;

	// 4. inc pointer
	adc_ma.ma_cnt++;
	if (adc_ma.ma_cnt >= ADC_MA_NUM)
		adc_ma.ma_cnt = 0;
	
}

#define		V_STEP		(3.3 / 4095.0)		// TL431 2.5V ref. output measured, it shows 2.494 V
void adc_calc_V_mW()
{
	int		i;
	
	// 1. calc. ADC_val --> vtg
	//    . V_STEP(1-bit resolution) = 2.5 V(vtg ref.) / 1024 (10-bit ADC) = 0.00244140625
	//    . vtg = V_STEP x adc_val
	adc_ma.ma_mV = V_STEP * adc_ma.ma_avg;
	
	// 2. calc. vtg --> mW
	//    . 2.4V = 10mW,  therfore x mW = (10mW x vtg) / 2.4
	// adc_ma.ma_mW = adc_ma.ma_mV * 10.0 / 2.4;
	
	// print avg, raw data
	if ((tick_10ms % TICK_2S) != 0)
		return;

	Serial.print("Avg=");
	Serial.print(adc_ma.ma_avg);
	Serial.print(", ");
	Serial.print(adc_ma.ma_mV, 3);
	Serial.print("V, ");
	// Serial.print(adc_ma.ma_mW, 3);
	// Serial.print("mW, raw=");
	for (i = 0; i < ADC_MA_NUM; i++)
	{
		Serial.print(" ");
		Serial.print(adc_ma.ma_raw[i]);
	}
	Serial.println(" ");
}


void key_svc()
{
	key.kin = digitalRead(BTN);

	switch (key.st)
	{
		case KEY_ST0_RLS :
			if (key.kin == KEY_PRS)
			{
				key.st = KEY_ST1_PRS_CHK;
				key.prs_st_time = tick_10ms;
			}
			break;
		
		case KEY_ST1_PRS_CHK :
			if (key.kin == KEY_PRS)
			{
				if (tick_10ms >= (key.prs_st_time + KEY_DEBOUNCE))
					key.st = KEY_ST2_PRS;
			}
			else
				key.st = KEY_ST0_RLS;
			
			break;
		
		case KEY_ST2_PRS :
			if (key.kin == KEY_PRS)
			{
				if (tick_10ms >= (key.prs_st_time + KEY_LONG_PRS))
				{
					key.st = KEY_ST4_LONG_PRS;
					OLED_disp_bottom_3_ver();
				}
			}
			else
			{
				key.st = KEY_ST3_RLS_CHK;
				key.rls_st_time = tick_10ms;
			}
			break;
		
		case KEY_ST3_RLS_CHK :
			if (key.kin == KEY_RLS)
			{
				if (tick_10ms >= (key.rls_st_time + KEY_DEBOUNCE))
				{
					key.st = KEY_ST0_RLS;
					
					// pressed key service 
					// if (disp_mode == MODE_flow)
					// 	disp_mode = MODE_temp;
					// else
					// 	disp_mode = MODE_flow;
					// update changed state to OLED
					OLED_disp_bottom_2();
				}
			}
			else
				key.st = KEY_ST2_PRS;
			
			break;
		
		case KEY_ST4_LONG_PRS :
			if (key.kin == KEY_RLS)
			{
				key.st = KEY_ST5_LONG_RLS_CHK;
				key.rls_st_time = tick_10ms;
			}
			break;
		
		case KEY_ST5_LONG_RLS_CHK :
			if (key.kin == KEY_RLS)
			{
				if (tick_10ms >= (key.rls_st_time + KEY_DEBOUNCE))
				{
					key.st = KEY_ST0_RLS;
					
					OLED_disp_bottom_3();
				}
			}
			else
				key.st = KEY_ST4_LONG_PRS;
			
			break;
		
		default :
			key.st = KEY_ST0_RLS;
			break;
	}
}

// display service
void OLED_svc()
{
	// .. at every 100msec
	if ((tick_10ms % TICK_100MS) != 1)
		return;

	OLED_disp_middle(disp_mode);
}


// display top line
void OLED_disp_top()
{
	display.drawLine(0,0,127,0, SH110X_WHITE);
	display.drawLine(0,0,0,8, SH110X_WHITE);
	display.drawLine(127,0,127,8, SH110X_WHITE);
	display.setFont();
	display.setCursor(1, 1);     // Start at top-left corner
	display.setTextColor(SH110X_BLACK, SH110X_WHITE); // Draw white text
	display.setTextSize(1);      // Normal 1:1 pixel scale

//                   123456789012345678901
	// display.println("  Laser Power Meter  ");
  display.println("  Pump & Flow meter  ");

	display.display();
}

// display bottom line, left
void OLED_disp_bottom_1()
{
	display.drawLine(0,55,42,55, SH110X_WHITE);
	display.drawLine(0,55,0,63, SH110X_WHITE);
	display.setFont();
	display.setCursor(1, 56);     
	display.setTextColor(SH110X_BLACK, SH110X_WHITE); 
	display.setTextSize(1);      

	//               1234567
	display.print  (" 635nm ");

	display.display();
}

// display bottom line, center
void OLED_disp_bottom_2()
{
	// 1. clear bottom 2 area first
	// display.fillRect(45, 55, 44, 9, SH110X_BLACK);
	
	// 2. write bottom 2
  display.drawLine(0,55,88,55, SH110X_WHITE);
	display.drawLine(0,55,0,63, SH110X_WHITE);
  display.drawLine(85,55,85,63, SH110X_WHITE);
  display.drawLine(86,55,86,63, SH110X_WHITE);
  display.drawLine(87,55,87,63, SH110X_WHITE);
	display.drawLine(88,55,88,63, SH110X_WHITE);
	// display.drawLine(45,55,88,55, SH110X_WHITE);
	// display.drawLine(45,55,45,63, SH110X_WHITE);
	// display.drawLine(88,55,88,63, SH110X_WHITE);
	display.setFont();
	display.setCursor(1, 56);     // Start at top-left corner
	display.setTextColor(SH110X_BLACK, SH110X_WHITE); // Draw white text
	display.setTextSize(1);      // Normal 1:1 pixel scale

	// if (disp_mode == MODE_flow)
	// //                 1234567
	// 	display.print("   mW  ");
	// else
	// //                 1234567
	// 	display.print("    V  ");

  if (slf3s_status == NOT_CONNECTED)
    display.print(" Not connected");
  else
  {
    if (product_num == SLF3S_4000B)
      display.print("  SLF3S-4000B ");
    else if (product_num == SLF3S_1300F)
      display.print("  SLF3S-1300F ");
    else
      display.print("    UNKNOWN   ");
  }

	display.display();
}

// display bottom line, right
void OLED_disp_bottom_3()
{
	display.drawLine(91,55,127,55, SH110X_WHITE);
  // display.drawLine(89,55,89,63, SH110X_WHITE);
  // display.drawLine(90,55,90,63, SH110X_WHITE);
	display.drawLine(91,55,91,63, SH110X_WHITE);
	display.setFont();
	display.setCursor(92, 56);     // Start at top-left corner
	display.setTextColor(SH110X_BLACK, SH110X_WHITE); // Draw white text
	display.setTextSize(1);      // Normal 1:1 pixel scale

	//               123456
	display.print  (" ATIK ");

	display.display();
}

// display bottom line, right
void OLED_disp_bottom_3_ver()
{
	display.drawLine(91,55,127,55, SH110X_WHITE);
	display.drawLine(91,55,91,63, SH110X_WHITE);
	display.setFont();
	display.setCursor(92, 56);     // Start at top-left corner
	display.setTextColor(SH110X_BLACK, SH110X_WHITE); // Draw white text
	display.setTextSize(1);      // Normal 1:1 pixel scale

	//               123456
	display.print  ("ver1.0");

	display.display();
}

// display bottom line
void OLED_disp_bottom()
{
	// OLED_disp_bottom_1();
	OLED_disp_bottom_2();
	OLED_disp_bottom_3();
}

// display middle line
// .. mode = 0 (mW), 1 (V)
// .. fval_main/sub = display value, float value, x.xxx format
//    in mW case : 0.000 .. 9.999 mW
//    in  V case : 0.000 .. 2.500  V
void OLED_disp_middle(unsigned char mode)
{
	uint16_t	fval_main;
	uint16_t	fval_sub;
  uint16_t	fval_sub2;
	unsigned char	overflow = 0;
	
	// 1. erase entire middle display area
	display.fillRect(0, 12, 128, 42, SH110X_BLACK);

	// 2. display main & sub display area ...
	// 2-1) display big main digit area, overflow(>) or not
	if (mode == MODE_flow)
	{
		// fval_main = adc_ma.ma_mW;
    fval_main = value_flow;
		fval_sub  = round(adc_ma.ma_mV / 3.3 * 100.0); // 3.3V = 100%
    fval_sub2 = value_temp;
		
		// if (fval_main >= 10.0)
		// 	overflow = 1;
	}
	else // (mode == MODE_temp)
	{
		fval_main = value_temp;
		// fval_sub  = adc_ma.ma_mW;
    fval_sub  = adc_ma.ma_mV;
		
		// if (fval_main >= (V_STEP * 4090))
		// 	overflow = 1;
	}
	
	if (overflow == 1)
	{
		display.setFont(&FreeSans18pt7b);
		display.setCursor(0, 35);
		display.setTextColor(SH110X_WHITE); // Draw white text
		display.setTextSize(1);
		
		display.print(">");
	}
	 
	// 2-2) display big main digits, ex) 9.123
	// display.setFont(&FreeSans18pt7b);
	// display.setCursor(22, 37);     
	// display.setTextColor(SH110X_WHITE); 
	// display.setTextSize(1);      

  display.setFont(&FreeSans12pt7b);
	display.setCursor(25, 37);     
	display.setTextColor(SH110X_WHITE); 
	display.setTextSize(1);      
	
  if (fval_main > 999)
    display.print(" ");
  else if (fval_main > 99)
    display.print("  ");
  else if (fval_main > 9)
    display.print("   ");
  else
    display.print("    ");

	display.print(fval_main);

  display.setFont(&FreeSans9pt7b);
	display.setCursor(65, 37);     

  display.print("ml/min");
	
	// 2-3) display sub small digits, ex) 9.123
	display.setFont();
	display.setCursor(5, 45);     
	display.setTextColor(SH110X_WHITE); 
	display.setTextSize(1);

  display.print("PUMP");

  if (fval_sub > 99)
    display.print(" ");
  else if (fval_sub > 9)
    display.print("  ");
  else
    display.print("   ");

	display.print(fval_sub);
	
	// 2-4) display sub unit, mW or V
	// if (mode == MODE_flow)
	// 	display.print(" V");
	// else
	// 	display.print(" mW");

  display.write(0x25);

	// 2-5) display main unit, mW or mV
	// display.setFont(&FreeSans9pt7b);
  display.setFont();
	display.setCursor(70, 45);     
	display.setTextColor(SH110X_WHITE); 
	display.setTextSize(1);
	
	// if (mode == MODE_flow)
	// 	display.print("mW");
	// else
	// 	display.print("   V");

  display.print("TEMP");
  
  if (fval_sub2 > 9)
    display.print(" ");
  else
    display.print("  ");

  display.print(fval_sub2);

  display.write(0xf7);
  display.print("C");

	display.display();
}