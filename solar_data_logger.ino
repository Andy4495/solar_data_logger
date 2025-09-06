/* Solar Power Data Logger
   https://github.com/Andy4495/solar_data_logger
   MIT License

   Use MspTandV library to take voltage measurements at regular
   intervals and store the data in non-volatile internal FRAM.

   Vcc plus a raw voltage are measured. I use the raw voltage
   to measure the the solar cell output through a resistor
   divider before it goes to the 3.3V voltage regulator.

   By storing the data in FRAM, the sketch can be run without 
   being connected to a computer and the data can be retrieved 
   at a later time. 

   Designed for use with MSP430FR2433 powered with a solar panel, 
   but could be adapted to other boards (particularly other "FR" 
   variants of the MSP430), and other power sources or data
   collection needs.

   Holding down PUSH1 at reboot prints the stored data over the
   serial port.
   Holding down PUSH2 at reboot clears the data store in FRAM. 
   - THE_LED is briefly flashed when FRAM is cleared.

   If a reboot is detected (e.g., due to loss of power from the 
   solar panel), then 5555 is stored in the next data cell (since
   that value is easy to spot and much greater than the ~3300 mV that 
   can be measured by the MSP430).

   THE_LED is flashed every few seconds (per the LED_TIME #define). Since 
   this takes a little extra power, it may be useful to remove the LED
   jumper after confirming that the program is running so that the 
   measurements aren't affected by the LED current draw.

   V_DIV_SCALE_FACTOR is the ratio of the voltage divider used between
   the raw voltage and ground. For example if the divider is connected as:
     Vraw <---> 10KOhm <---> RAW_ADC_PIN <---> 5KOhm <---> GND
   (meaning 10K and 5K divider resistors), then the scaling factor is
   5000/(5000 + 10000) = 0.3333. 

   Uses external library https://github.com/Andy4495/MspTandV
*/
/* Version History
   10/19/2023   Andy4495    Original
   09-JUL-2025  Andy4495    Add print calculated voltage when dumping FRAM
   30-AUG-2025  Andy4495    Update scale factor to allow readings up to about 6 V

*/

#include "MspTandV.h"

#define LOOP_TIME 60000UL           // Milliseconds between voltage readings
#define LED_TIME 3000UL             // Time between LED toggling
#define NUM_SAMPLES (1024 + 512)    // 1536 total readings at 60 second intervals stores 25h36m of data (even more since no power at night)
#define REBOOT_DETECTED 5555
#define V_DIV_SCALE_FACTOR 0.24605  // We are going to use floating point math for this, since we have the space
#define RAW_ADC_PIN 5               // Analog pin used to measure the raw voltage (typically through a voltage divider)
#define LED_PWM_LEVEL 32            // Controls LED brightness, higher is brighter (and uses more current)
#if defined(__MSP430FR6989__)       // The FR6989 does not define LED2, so use GREEN_LED instead
#define THE_LED GREEN_LED
#else
#define THE_LED LED2
#endif

MspVcc myVcc;
MspAdc myAdc(RAW_ADC_PIN, 1);  // Using internal voltage reference 1, which is 1.5V on FR2433. See MspTandV library if using a different processor.

uint16_t first_time PLACE_IN_FRAM;
uint16_t loop_counter PLACE_IN_FRAM;
uint16_t vcc[NUM_SAMPLES] PLACE_IN_FRAM;
uint16_t adc[NUM_SAMPLES] PLACE_IN_FRAM;

unsigned long prev_millis = 0;
unsigned long led_millis = 0;
unsigned long current_millis;
float divided_mv;
int led_state = 0;

void setup() {
  Serial.begin(9600);
#if defined(__MSP430FR2433__)
  SYSCFG0 = FRWPPW;  // Write enable data and program FRAM
#endif
  pinMode(THE_LED, OUTPUT);
  digitalWrite(THE_LED, LOW);

  pinMode(PUSH2, INPUT_PULLUP);
  // Check if this is the first time through
  // Either look for a "magic number" (0x49) or PUSH2 pressed at reset
  if ((first_time != 0x49) || (digitalRead(PUSH2) == LOW)) {
    Serial.println("Clearing FRAM, start logging at 0.");
    for (int i = 0; i < NUM_SAMPLES; i++) {
      vcc[i] = 0;
      adc[i] = 0;
    }
    loop_counter = 0;
    first_time = 0x49;
    // Briefly flash LED to indicate memory clear
    digitalWrite(THE_LED, HIGH);
    delay(500);
    digitalWrite(THE_LED, LOW);
  } else {  // If there was a reboot, put the REBOOT_DETECTED value in the next slot
    vcc[loop_counter++] = REBOOT_DETECTED;
    if (loop_counter >= NUM_SAMPLES) loop_counter = 0;
    Serial.print("Reboot detected. start logging at: ");
    Serial.println(loop_counter);
  }

  pinMode(PUSH1, INPUT_PULLUP);
  // Dump the FRAM to Serial if PUSH1 is pressed at bootup
  if (digitalRead(PUSH1) == LOW) {
  Serial.print("i, vcc[i], adc[i], V_calc: ");
  for (int i = 0; i < NUM_SAMPLES; i++) {
      Serial.print(i);
      Serial.print(", ");
      Serial.print(vcc[i]);
      Serial.print(", ");
      Serial.print(adc[i]);
      Serial.print(", ");
      Serial.print(adc[i] / (float)ADC_STEPS * (float)VCC_REF1_DV / V_DIV_SCALE_FACTOR / 10.0, 6);
      Serial.println("");
    }
  }
}

void loop() {
  current_millis = millis();
  if (current_millis - prev_millis > LOOP_TIME) {
    prev_millis = millis();
    myVcc.read(CAL_ONLY);
    vcc[loop_counter] = myVcc.getVccCalibrated();
    Serial.print("Loop, Vcc, ADC_Calibrated, V_Calculated: ");
    Serial.print(loop_counter);
    Serial.print(" , ");
    myVcc.read(CAL_ONLY);
    Serial.print(vcc[loop_counter]);
    Serial.print(", ");
    myAdc.read();
    adc[loop_counter] = myAdc.getAdcCalibrated();
    Serial.print(adc[loop_counter]);
    Serial.print(", ");
    divided_mv = adc[loop_counter] / (float)ADC_STEPS * (float)VCC_REF1_DV / V_DIV_SCALE_FACTOR / 10.0;
    Serial.print(divided_mv, 6);
    Serial.println("");
    loop_counter++;
    if (loop_counter >= NUM_SAMPLES) loop_counter = 0;
  }

  // Flash the LED periodically, but low power using PWM
  if (current_millis - led_millis > LED_TIME) {
    led_millis = current_millis;
    if (led_state == 0) analogWrite(THE_LED, LED_PWM_LEVEL);
    else analogWrite(THE_LED, 0);
    led_state = ~led_state;
  }
}
