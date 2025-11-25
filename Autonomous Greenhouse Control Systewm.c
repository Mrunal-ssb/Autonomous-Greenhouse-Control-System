#include <stdint.h>

#include <string.h>

#include <stdlib.h>

#include <time.h>  // For srand(time(NULL))

#include "stm32f405xx.h"

#include "lcd.h"

#define LED_PING   5  // Green LED pin (PA5)

#define LED_PINR   6  // Red LED pin (PA6)

#define CLK_PIN     (1 << 6)  // PB6

#define DIO_PIN     (1 << 7)  // PB7

#define A1    3

#define A2    4

#define B1  9

#define B2  10

#define DHT11_PORT GPIOA

#define DHT11_PIN  2

uint16_t adc;

#define L1 3

#define L2 4

#define L3 5

#define L4 8

#define L5 9

#define L6 10

#define L7 11

#define L8 11

#define L9 12

#define L10 13

int temperature=0;

int humidity=0;

int ret=0;

// Function prototypes

void TIM3_Init(void);

// Function prototypes

void TIM2_Init(void);

void delayMs(uint32_t ms);

void delay_us(uint32_t us);

void init_gpio(void);

void TM_start(void);

void TM_stop(void);

void TM_writeByte(uint8_t b);

void TM_displayDigits(const uint8_t digits[4]);

char scan_keypad(void);

void gpio_keypad_init(void);

void DHT11_Init(void);

void DHT11_SetPinOutput(void);

void DHT11_SetPinInput(void);

uint8_t DHT11_ReadBit(void);

int DHT11_Read(uint8_t *humidity, uint8_t *temperature);

const uint8_t digitMap[10] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};

const char keymap[4][4] = {{'1','2','3','A'},{'4','5','6','B'},{'7','8','9','C'},{'*','0','#','D'}};

const uint8_t row_pins[4] = {10, 1, 3, 8};  // PC0, PC1, PC3, PC8

const uint8_t col_pins[4] = {4, 5, 6, 7};  // PC4, PC5, PC6, PC7


void init_gpio(void) {

RCC->AHB1ENR |= (1 << 1); // Enable GPIOA and GPIOB clocks

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

GPIOA->MODER |= (1 << (LED_PING * 2));       // Output mode (01)

GPIOA->OTYPER &= ~(1 << LED_PING);           // Push-Pull

GPIOA->MODER |= (1 << (LED_PINR * 2));       // Output mode (01)

GPIOA->OTYPER &= ~(1 << LED_PINR);

// PB6 (CLK), PB7 (DIO) as output

GPIOB->MODER &= ~((3 << 12) | (3 << 14));  // Clear mode bits

GPIOB->MODER |=  ((1 << 12) | (1 << 14));  // Set to output

GPIOB->OTYPER &= ~(1 << 6); // PB6: push-pull

GPIOB->OTYPER |=  (1 << 6); // PB7: open-drain

}

// Globals

uint8_t currentRandomCode[4];  // Holds current 4-digit random code (digits as 0-9)

void TIM2_Init(void) {

RCC->APB1ENR |= (1 << 0); // Enable TIM2 clock

TIM2->PSC = 16 - 1;       // 1 µs tick

TIM2->ARR = 0xFFFF;

TIM2->CR1 |= (1 << 0);    // Enable timer

}

void delay_us(uint32_t us) {

TIM2->CNT = 0;

while (TIM2->CNT < us);

}

void delayMs(uint32_t ms) {

for (uint32_t i = 0; i < ms; i++) {

delay_us(1000);

}

}

void generateRandomCode(void) {

for (int i = 0; i < 4; i++) {

currentRandomCode[i] = rand() % 10;

}

}

void TM_start(void) {

GPIOB->BSRR = CLK_PIN | DIO_PIN;

delay_us(15);

GPIOB->BSRR = (DIO_PIN << 16); // DIO low

delay_us(15);

GPIOB->BSRR = (CLK_PIN << 16); // CLK low

}

void TM_stop(void) {

GPIOB->BSRR = (CLK_PIN << 16); // CLK low

GPIOB->BSRR = (DIO_PIN << 16); // DIO low

delay_us(15);

GPIOB->BSRR = CLK_PIN;

delay_us(15);

GPIOB->BSRR = DIO_PIN;

}

void TM_writeByte(uint8_t b) {

for (int i = 0; i < 8; i++) {

GPIOB->BSRR = (CLK_PIN << 16); // CLK low

if (b & 0x01)

GPIOB->BSRR = DIO_PIN;

else

GPIOB->BSRR = (DIO_PIN << 16);

delay_us(15);

GPIOB->BSRR = CLK_PIN; // CLK high

delay_us(15);

b >>= 1;

}

// ACK (skip reading)

GPIOB->MODER &= ~(3 << 14);  // PB7 as input

GPIOB->BSRR = (CLK_PIN << 16); // CLK low

delay_us(50);

GPIOB->BSRR = CLK_PIN;        // CLK high

delay_us(50);

GPIOB->BSRR = (CLK_PIN << 16); // CLK low

GPIOB->MODER |= (1 << 14);    // PB7 as output

}

void TM_displayDigits(const uint8_t digits[4]) {

TM_start();

TM_writeByte(0x40);  // auto-increment mode

TM_stop();

TM_start();

TM_writeByte(0xC0);  // Start address 0

for (int i = 0; i < 4; i++) {

TM_writeByte(digitMap[digits[i]]);

}

TM_stop();

TM_start();

TM_writeByte(0x88 | 0x07);  // Display ON, brightness max

TM_stop();

}

void gpio_keypad_init(void) {

RCC->AHB1ENR |= (1 << 2);  // Enable Port C clock

for (int i = 0; i < 4; i++) {

GPIOC->MODER &= ~(3 << (row_pins[i] * 2));  // Clear mode

GPIOC->MODER |=  (1 << (row_pins[i] * 2));  // Set as output

}

// Set columns (PC4–PC7) as input with pull-up

for (int i = 0; i < 4; i++) {

GPIOC->MODER &= ~(3 << (col_pins[i] * 2));   // Input mode

GPIOC->PUPDR &= ~(3 << (col_pins[i] * 2));   // Clear pull

GPIOC->PUPDR |=  (1 << (col_pins[i] * 2));   // Pull-up

}

}

char scan_keypad(void) {

for (int row = 0; row < 4; row++) {

// Set all rows HIGH

for (int r = 0; r < 4; r++)

GPIOC->ODR |= (1 << row_pins[r]);

// Pull current row LOW

GPIOC->ODR &= ~(1 << row_pins[row]);

for (volatile int d = 0; d < 1000; d++); // short delay

for (int col = 0; col < 4; col++) {

if ((GPIOC->IDR & (1 << col_pins[col])) == 0) {

return keymap[row][col];  // Key matched

}

}

}

return 0;  // No key pressed

}


void TIM3_Init(void) {

RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Enable TIM3 clock

TIM3->PSC = 16 - 1;                  // Prescaler: 16 -> 1 MHz timer frequency (1 µs per tick)

TIM3->ARR = 0xFFFF;                  // Auto-reload: max value

TIM3->CR1 |= TIM_CR1_CEN;           // Enable the counter

}


void DHT11_Init(void) {

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

GPIOA->MODER |= (1 << (A1* 2));      // Output mode (01)

GPIOA->OTYPER &= ~(1 << A1);          // Push-Pull

GPIOA->MODER |= (1 << (A2 * 2));      // Output mode (01)

GPIOA->OTYPER &= ~(1 << A2);          // Push-Pull

GPIOA->MODER |= (1 << (B1 * 2));      // Output mode (01)

GPIOA->OTYPER &= ~(1 << B1);

GPIOA->MODER |= (1 << (B2 * 2));      // Output mode (01)

GPIOA->OTYPER &= ~(1 << B2);

DHT11_PORT->MODER &= ~(3 << (DHT11_PIN * 2));

DHT11_PORT->MODER |= (1 << (DHT11_PIN * 2));  // output mode

DHT11_PORT->OTYPER |= (1 << DHT11_PIN);       // open-drain

DHT11_PORT->PUPDR &= ~(3 << (DHT11_PIN * 2));

DHT11_PORT->PUPDR |= (1 << (DHT11_PIN * 2));  // pull-up

DHT11_PORT->BSRR = (1 << DHT11_PIN);          // set high

}

void DHT11_SetPinOutput(void) {

DHT11_PORT->MODER &= ~(3 << (DHT11_PIN * 2));

DHT11_PORT->MODER |= (1 << (DHT11_PIN * 2));  // output mode

}

void DHT11_SetPinInput(void) {

DHT11_PORT->MODER &= ~(3 << (DHT11_PIN * 2)); // input mode

}

uint8_t DHT11_ReadBit(void) {

while (!(DHT11_PORT->IDR & (1 << DHT11_PIN)));

delay_us(40);

if (DHT11_PORT->IDR & (1 << DHT11_PIN)) {

while (DHT11_PORT->IDR & (1 << DHT11_PIN));

return 1;

}

return 0;

}

int DHT11_Read(uint8_t *humidity, uint8_t *temperature) {

uint8_t bits[5] = {0,0,0,0,0};

uint32_t timeout;

DHT11_SetPinOutput();

DHT11_PORT->BSRR = (1 << (DHT11_PIN + 16)); // pull low

delay_us(18000);

DHT11_PORT->BSRR = (1 << DHT11_PIN);        // pull high

delay_us(30);

DHT11_SetPinInput();

timeout = 0;

while ((DHT11_PORT->IDR & (1 << DHT11_PIN)) != 0) if (++timeout > 10000) return -1;

timeout = 0;

while ((DHT11_PORT->IDR & (1 << DHT11_PIN)) == 0) if (++timeout > 10000) return -1;

timeout = 0;

while ((DHT11_PORT->IDR & (1 << DHT11_PIN)) != 0) if (++timeout > 10000) return -1;

for (int i = 0; i < 40; i++) {

bits[i/8] <<= 1;

bits[i/8] |= DHT11_ReadBit();

}

// Verify checksum

if ((uint8_t)(bits[0] + bits[1] + bits[2] + bits[3]) != bits[4]) {

return -2;

}

*humidity = bits[0];

*temperature = bits[2];

return 0;

}

void adc_init(void){

RCC->AHB1ENR |= (1 << 2);  // Enable GPIOC Clock

 
// Set PC0 to analog mode (MODER bits for pin 0: bits 1 and 0)

GPIOC->MODER |= (1 << 0) | (1 << 1);  // PC0 in analog mode

RCC->APB2ENR |= (1 << 8);  // Enable ADC1 Clock

ADC1->SQR3 &= ~(0x1F << 0);   // Clear first channel bits

ADC1->SQR3 |= (10 << 0);      // Channel 10 (PC0)

ADC1->CR1 = (1 << 8);  // Scan Mode

ADC1->CR2 |= (1 << 1);  // Continuous Mode

ADC1->CR2 |= (1 << 0);  // ADC enable


}

uint16_t adc_read(void)

{

ADC1->CR2 |= (1 << 30);  // Start conversion

while (!(ADC1->SR & (1 << 1)));  // Wait until EOC (End of Conversion)

return ADC1->DR;

}

void ldr_bargraph(void){

RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

GPIOB->MODER |= (1 << (L1 * 2));       // Output mode (01)

GPIOB->OTYPER &= ~(1 << L1);           // Push-Pull

GPIOB->MODER |= (1 << (L2 * 2));       // Output mode (01)

GPIOB->OTYPER &= ~(1 << L2);

GPIOB->MODER |= (1 << (L3 * 2));       // Output mode (01)

GPIOB->OTYPER &= ~(1 << L3);

GPIOB->MODER |= (1 << (L4 * 2));       // Output mode (01)

GPIOB->OTYPER &= ~(1 << L4);

GPIOB->MODER |= (1 << (L5 * 2));       // Output mode (01)

GPIOB->OTYPER &= ~(1 << L5);           // Push-Pull

GPIOB->MODER |= (1 << (L6 * 2));       // Output mode (01)

GPIOB->OTYPER &= ~(1 << L6);

GPIOB->MODER |= (1 << (L7 * 2));       // Output mode (01)

GPIOB->OTYPER &= ~(1 << L7);

GPIOC->MODER |= (1 << (L8 * 2));       // Output mode (01)

GPIOC->OTYPER &= ~(1 << L8);

GPIOC->MODER |= (1 << (L9 * 2));       // Output mode (01)

GPIOC->OTYPER &= ~(1 << L9);           // Push-Pull

GPIOC->MODER |= (1 << (L10* 2));       // Output mode (01)

GPIOC->OTYPER &= ~(1 << L10);

}

int main(void) {

// Initialize hardware

LcdInit();

gpio_keypad_init();

init_gpio();

TIM2_Init();

char buf[32];

TIM3_Init();

DHT11_Init();

adc_init();

ldr_bargraph();

// Seed random generator once

srand(time(NULL));

// Show welcome message at power on

LcdFxn(0, 0x01);  // Clear LCD

lprint(0x80, "Welcome");

// Turn off both LEDs initially

GPIOA->ODR &= ~(1 << LED_PING);  // Green LED OFF

GPIOA->ODR &= ~(1 << LED_PINR);  // Red LED OFF

generateRandomCode();

TM_displayDigits(currentRandomCode);

static uint32_t lastTimerCount = 0;

static uint32_t overflowCount = 0;

uint32_t lastMillis = 0;

uint32_t getMillis(void) {

uint32_t currentCount = TIM2->CNT;

if (currentCount < lastTimerCount) {

overflowCount++;

}

lastTimerCount = currentCount;

uint32_t totalTicks = (overflowCount << 16) + currentCount;

return totalTicks / 1000;  // Convert us to ms

}

char passkey[5] = {0};   // To store entered code (4 chars + null)

const int codeLength = 4;


while (1) {

uint32_t currentMillis = getMillis();

if (currentMillis - lastMillis >= 18000) {

generateRandomCode();

TM_displayDigits(currentRandomCode);

lastMillis = currentMillis;

}

char key = scan_keypad();

if (key == '*') {

LcdFxn(0, 0x01);

lprint(0x80, "Security Code");

memset(passkey, 0, sizeof(passkey));

int i = 0;

// Turn off LEDs while entering

GPIOA->ODR &= ~(1 << LED_PING);

GPIOA->ODR &= ~(1 << LED_PINR);

while (1) {

key = scan_keypad();

if (key) {

if (key == '#') {

passkey[i] = '\0'; // Null terminate

LcdFxn(0, 0x01);


// Compare entered passkey with currentRandomCode

int match = 1;

for (int j = 0; j < codeLength; j++) {

if (passkey[j] - '0' != currentRandomCode[j]) {

match = 0;

break;

}

}

if (match) {

lprint(0x80, "Access Granted");

GPIOA->ODR |= (1 << LED_PING);   // Green LED ON

GPIOA->ODR &= ~(1 << LED_PINR);  // Red LED OFF

delayMs(2000);

LcdFxn(0, 0x01);

while(1)

{

int ret = DHT11_Read((uint8_t *)&humidity, (uint8_t *)&temperature);

if (ret == 0) {

sprintf(buf, "Hum:%d%% Temp:%dC", humidity,temperature);

lprint(0x00, buf);   // Print humidity on line 1

if(temperature>27)

{GPIOA->ODR |= (1 << A1);

GPIOA->ODR &= ~(1 << A2);

lprint(0xCc,"mildly hot");

LcdFxn(0, 0x01);

}

else{

GPIOA->ODR &= ~(1 << A1);

GPIOA->ODR &= ~(1 << A2);

lprint(0xCc,"cool");

}

if(humidity<50)

{GPIOA->ODR &= ~ (1 << B1);   // LED1 ON

GPIOA->ODR &= ~(1 << B2);    // LED2 ON


}

else if(humidity >= 57 && humidity <= 60){

GPIOA->ODR |=(1 << B1);   // LED1 ON

GPIOA->ODR &= ~(1 << B2);    // LED2 ON

}

else if(humidity>62)

{GPIOA->ODR |= (1 << B1);   // LED1 ON

GPIOA->ODR |=(1 << B2);    // LED2 ON

}

adc = adc_read();

// Control LEDs based on temperature

if (adc >= 0 && adc <= 800)

{GPIOB->ODR&= ~(1 << L1);   // LED1 ON

GPIOB->ODR &= ~(1 << L2);   // LED2 ON

GPIOB->ODR &= ~(1 << L3);  // LED1 OFF

GPIOB->ODR &= ~(1 << L4);

GPIOB->ODR &= ~(1 << L5);

GPIOB->ODR &= ~(1 << L6);

GPIOB->ODR &= ~(1 << L7);

GPIOC->ODR &= ~(1 << L8);

GPIOC->ODR &= ~(1 << L9);

GPIOC->ODR &= ~(1 << L10);

lprint(0xC0, "100");

}

else if(adc >= 900 && adc <= 1100)

{

GPIOB->ODR |= (1 << L1);   // LED1 ON

GPIOB->ODR |= (1 << L2);   // LED2 ON

GPIOB->ODR |=(1 << L3);  // LED1 OFF

GPIOB->ODR |=(1 << L4);

GPIOB->ODR &= ~(1 << L5);

GPIOB->ODR &= ~(1 << L6);

GPIOB->ODR &= ~(1 << L7);

GPIOC->ODR &= ~(1 << L8);

GPIOC->ODR &= ~(1 << L9);

 
GPIOC->ODR &= ~(1 << L10);

lprint(0xC0, "70");

}

else if (adc >= 1200 && adc <= 1800)


{GPIOB->ODR |= (1 << L1);   // LED1 ON

GPIOB->ODR |= (1 << L2);   // LED2 ON

GPIOB->ODR |=(1 << L3);  // LED1 OFF

GPIOB->ODR |=(1 << L4);

GPIOB->ODR |=(1 << L5);

GPIOB->ODR &= ~(1 << L6);

GPIOB->ODR &= ~(1 << L7);

GPIOC->ODR &= ~(1 << L8);

GPIOC->ODR &= ~(1 << L9);

GPIOC->ODR &= ~(1 << L10);

lprint(0xC0, "50");

}

else if (adc >= 2000 && adc <= 2500)

{GPIOB->ODR |= (1 << L1);   // LED1 ON

GPIOB->ODR |= (1 << L2);   // LED2 ON

GPIOB->ODR |=(1 << L3);  // LED1 OFF

GPIOB->ODR |=(1 << L4);

GPIOB->ODR |=(1 << L5);

GPIOB->ODR |=(1 << L6);

GPIOB->ODR |=(1 << L7);

GPIOC->ODR &= ~(1 << L8);

GPIOC->ODR &= ~(1 << L9);

GPIOC->ODR &= ~(1 << L10);

lprint(0xC0, "30");

}

else if (adc >= 2600 && adc <= 3500)

{GPIOB->ODR |= (1 << L1);   // LED1 ON

GPIOB->ODR |= (1 << L2);   // LED2 ON

GPIOB->ODR |=(1 << L3);  // LED1 OFF

GPIOB->ODR |=(1 << L4);

GPIOB->ODR |=(1 << L5);

GPIOB->ODR |=(1 << L6);

GPIOB->ODR |=(1 << L7);

GPIOC->ODR |=(1 << L8);

GPIOC->ODR |=(1 << L9);

GPIOC->ODR |=(1 << L10);

lprint(0xC0, "10");

}

delayMs(2000);

LcdFxn(0, 0x01);

}


}


} else {

lprint(0x80, "Access Denied");

lprint(0xC0, "Try Again!");

GPIOA->ODR |= (1 << LED_PINR);   // Red LED ON

GPIOA->ODR &= ~(1 << LED_PING);  // Green LED OFF

}

//  delayMs(3000);

LcdFxn(0, 0x01);

lprint(0x80, "Welcome");

break;

}

if (i < codeLength && key >= '0' && key <= '9') {

passkey[i++] = key;

lprint(0xC0 + i - 1, "*");

}

// Wait for key release

while (scan_keypad());

delayMs(100);

}

}

}

}

}
 