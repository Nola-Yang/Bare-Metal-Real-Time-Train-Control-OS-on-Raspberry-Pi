#include <stdint.h>
#include "rpi.h"

static char* const GPIO_BASE = (char*)(MMIO_BASE + 0x200000);

#define GPFSEL_REG(reg)             (*(volatile uint32_t*)(GPIO_BASE + reg * 4))
#define GPIO_PUP_PDN_CNTRL_REG(reg) (*(volatile uint32_t*)(GPIO_BASE + 0xe4 + reg * 4))

// GPIO interrupt registers
#define GPEDS_REG(reg) (*(volatile uint32_t*)(GPIO_BASE + 0x40 + reg * 4))
#define GPLEN_REG(reg) (*(volatile uint32_t*)(GPIO_BASE + 0x70 + reg * 4))
#define GPIO_GPEDS0   GPEDS_REG(0)  // Event Detect Status 0-31
#define GPIO_GPREN0   (*(volatile uint32_t*)(GPIO_BASE + 0x4C))  // Rising Edge Enable 0-31
#define GPIO_GPFEN0   (*(volatile uint32_t*)(GPIO_BASE + 0x58))  // Falling Edge Enable 0-31

// function control settings for GPIO pins
static const uint32_t GPIO_INPUT  = 0x00;
static const uint32_t GPIO_OUTPUT = 0x01;
static const uint32_t GPIO_ALTFN0 = 0x04;
static const uint32_t GPIO_ALTFN1 = 0x05;
static const uint32_t GPIO_ALTFN2 = 0x06;
static const uint32_t GPIO_ALTFN3 = 0x07;
static const uint32_t GPIO_ALTFN4 = 0x03;
static const uint32_t GPIO_ALTFN5 = 0x02;

// pup/pdn resistor settings for GPIO pins
static const uint32_t GPIO_NONE = 0x00;
static const uint32_t GPIO_PUP  = 0x01;
static const uint32_t GPIO_PDP  = 0x02;

static void setup_gpio(uint32_t pin, uint32_t setting, uint32_t resistor) {
	uint32_t reg   =  pin / 10;
	uint32_t shift = (pin % 10) * 3;
	uint32_t status = GPFSEL_REG(reg);    // read status
	status &= ~(7u << shift);             // clear bits
	status |=  (setting << shift);        // set bits
	GPFSEL_REG(reg) = status;

	reg   =  pin / 16;
	shift = (pin % 16) * 2;
	status = GPIO_PUP_PDN_CNTRL_REG(reg); // read status
	status &= ~(3u << shift);             // clear bits
	status |=  (resistor << shift);       // set bits
	GPIO_PUP_PDN_CNTRL_REG(reg) = status; // write back
}

// GPIO pins 14 & 15 already configured by boot loader, but redo for clarity.
void gpio_init() {
	setup_gpio( 8, GPIO_ALTFN0, GPIO_NONE); // SPI0_CE0_N
	setup_gpio( 9, GPIO_ALTFN0, GPIO_NONE); // SPI0_MISO
	setup_gpio(10, GPIO_ALTFN0, GPIO_NONE); // SPI0_MOSI
	setup_gpio(11, GPIO_ALTFN0, GPIO_NONE); // SPI0_SCLK

	setup_gpio(14, GPIO_ALTFN0, GPIO_NONE); // UART TXD0
	setup_gpio(15, GPIO_ALTFN0, GPIO_NONE); // UART RXD0

	// MCP2515 INT pin (active low)
	gpio_init_interrupt();
}

void gpio_enable_falling_edge_detect(uint32_t pin) {
	GPIO_GPFEN0 |= (1u << pin);
}

void gpio_clear_event(uint32_t pin) {
	GPIO_GPEDS0 = (1u << pin);  // Write 1 to clear
}

uint32_t gpio_check_event(uint32_t pin) {
	return (GPIO_GPEDS0 >> pin) & 1;
}

static void gpio_set_pin_low_detect(uint32_t pin, int enable) {
  uint32_t reg   = pin / 32;
  uint32_t shift = pin % 32;
  if (enable) GPLEN_REG(reg) |=  (1 << shift); // enable pin low detect
  else        GPLEN_REG(reg) &= ~(1 << shift); // disable pin low detect
}

uint32_t gpio_get_event_detect_status(uint32_t pin) {
  uint32_t reg = pin / 32;
  uint32_t shift = pin % 32;
  return (GPEDS_REG(reg) >> shift) & 0x01; // return the bit corresponding to the pin
}

void gpio_clr_event_detect_status(uint32_t pin) {
  uint32_t reg = pin / 32;
  uint32_t shift = pin % 32;
  GPEDS_REG(reg) = (1 << shift); // clear the event detect status for the pin
}

void gpio_init_interrupt() {
  setup_gpio(17, GPIO_INPUT,  GPIO_NONE); // configure MCP2515_INT pin
  gpio_set_pin_low_detect(17, 1);         // enable low detect on MCP2515_INT pin
}