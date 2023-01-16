/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

/* Movement Pins for manual drive */
#define PTD0_PIN 0 // T0C0
#define PTD1_PIN 1 // T0C1
#define PTD2_PIN 2 // T0C2
#define PTD3_PIN 3 // T0C3
#define PTE31_PIN 31 // T0C4
#define PTD5_PIN 5 // T0C5
#define PTB0_PIN 0 // T1C0
#define PTB1_PIN 1 // T1C1 
#define SPEED  6000 // 80% of 7500
#define MAXSPEED 7500
#define TURNSPEED 1125
#define SPINSPEED 4500

/* Movement pins for auto drive */
#define PTA4_PIN 4  // T0C1
#define PTA5_PIN 5  // T0C2
#define PTC8_PIN 8  // T0C4
#define PTC9_PIN 9  // T0C5

/* UART pins */
#define BAUD_RATE 9600	
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128 //Priority level in NVIC for UART interrupt 

/* GPIO pins */
//Green (Front) LED pins
#define PTC7_PIN				7
#define PTC0_PIN				0
#define PTC3_PIN				3
#define PTC4_PIN				4
#define PTC5_PIN				5
#define PTC6_PIN				6
#define PTC10_PIN				10
#define PTC11_PIN				11

//Red (Rear) LED pins
#define PTE5_PIN				5
#define PTE4_PIN				4
#define PTE3_PIN				3
#define PTE2_PIN				2
#define PTB11_PIN				11
#define PTB10_PIN				10
#define PTB9_PIN				9
#define PTB8_PIN				8

/* Audio pins and configs */
#define PTA1_PIN				 1   // T2C0
#define SONG_CNT				 7
#define TONE_CNT				 7
#define FREQ_2_MOD(x)		(375000 / x)
#define NOTE_C					 262
#define NOTE_D					 294
#define NOTE_E					 330
#define NOTE_F					 349
#define NOTE_G					 392
#define NOTE_A					 440
#define NOTE_B					 494	

/* Ultrasonic pins */
#define PTE20_PIN				20  // Trig pin - T1C0
#define PTE21_PIN				21 // Echo pin - T1C1
#define TPM1_INT_PRIO  192 //Priority level in NVIC for TPM1 interrupt - Lowest priority compared to UART_INT_PRIO

/* Masking */
#define FUNCTION_MASK(x) ((x) & 0xF0) 
#define MESSAGE_MASK(x) ((x) & 0x0F)
#define MOTOR_FUNCTION 0x20
#define WIFI_FUNCTION 0x10
#define AUTO_DRIVE_FUNCTION 0x50
#define MASK(x) (1 << (x))

/* Message Queue message count*/
#define MSG_COUNT 1

volatile int risingEdge = 1;

osThreadId_t motor_Id, brain_Id, audio_Id, led_Id, autoDrive_Id, ultrasonic_Id;
osMessageQueueId_t motorMsg, audioMsg, ledMsg, autoDriveMsg, utMsg, brainMsg,utReadingMsg;

const osThreadAttr_t brainPriority = {
    .priority = osPriorityHigh4
};

typedef struct {
	uint8_t cmd;
	uint16_t dist;
} myDataPkt;

typedef enum states {
	led_off,
	led_on,
} led_state_t;

typedef enum led_status {
	nothing,
	wifi_check,
	moving,
	stopped,
} status_checker_t;	

typedef enum audio_states {
	blank,
	unique_tune,     // Includes wifi tune and end of challenge tune
	challenge_tune, // Start of challenge tune
} audio_state_t;

typedef enum motor_states {
	forward = 1, backward, right, left, stop, staticRight, staticLeft, 
	backwardRight, backwardLeft, stopRun,
} direction_t;

typedef enum {
	start = 1,
	end,
} auto_state_t;

int song[] = {NOTE_C, NOTE_G, NOTE_A, NOTE_G, NOTE_F, NOTE_E, NOTE_D};

int uniqueTone[] = {NOTE_C, NOTE_D, NOTE_E, NOTE_F, NOTE_G, NOTE_A, NOTE_B};								
							
/* LED (GPIO) Initialization Function */
void initLEDs(void) {
	
	// Enable Clock to PORTB, PORTC and PORTE
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTC_MASK) | (SIM_SCGC5_PORTE_MASK));

	// Configure MUX settings to make the pins GPIO (Green (Front) LEDs)
  PORTC->PCR[PTC7_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[PTC7_PIN] |= PORT_PCR_MUX(1);
  PORTC->PCR[PTC0_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[PTC0_PIN] |= PORT_PCR_MUX(1);
  PORTC->PCR[PTC5_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[PTC5_PIN] |= PORT_PCR_MUX(1);
  PORTC->PCR[PTC6_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[PTC6_PIN] |= PORT_PCR_MUX(1);
  PORTC->PCR[PTC4_PIN] &= ~PORT_PCR_MUX_MASK;	
  PORTC->PCR[PTC4_PIN] |= PORT_PCR_MUX(1);
  PORTC->PCR[PTC10_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[PTC10_PIN] |= PORT_PCR_MUX(1);
  PORTC->PCR[PTC11_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[PTC11_PIN] |= PORT_PCR_MUX(1);
  PORTC->PCR[PTC3_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTC->PCR[PTC3_PIN] |= PORT_PCR_MUX(1);

	// Set Data Direction Registers for PortC
  PTC->PDDR |= (MASK(PTC7_PIN) | MASK(PTC0_PIN) | MASK(PTC5_PIN) | MASK(PTC6_PIN) | MASK(PTC4_PIN) | MASK(PTC10_PIN) | MASK(PTC11_PIN) | MASK(PTC3_PIN));

  // Configure MUX settings to make the pins GPIO (Red (Rear) LEDs)
  PORTE->PCR[PTE5_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[PTE5_PIN] |= PORT_PCR_MUX(1);
  PORTE->PCR[PTE4_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[PTE4_PIN] |= PORT_PCR_MUX(1);
  PORTE->PCR[PTE3_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[PTE3_PIN] |= PORT_PCR_MUX(1);
  PORTE->PCR[PTE2_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[PTE2_PIN] |= PORT_PCR_MUX(1);
  PORTB->PCR[PTB11_PIN] &= ~PORT_PCR_MUX_MASK;	
  PORTB->PCR[PTB11_PIN] |= PORT_PCR_MUX(1);
  PORTB->PCR[PTB10_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB10_PIN] |= PORT_PCR_MUX(1);
  PORTB->PCR[PTB9_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB9_PIN] |= PORT_PCR_MUX(1);
  PORTB->PCR[PTB8_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTB->PCR[PTB8_PIN] |= PORT_PCR_MUX(1);

  PTE->PDDR |= (MASK(PTE5_PIN) | MASK(PTE4_PIN) | MASK(PTE3_PIN) | MASK(PTE2_PIN));
  PTB->PDDR |= (MASK(PTB11_PIN) | MASK(PTB10_PIN) | MASK(PTB9_PIN) | MASK(PTB8_PIN));	
	
}

/* init UART2 */
void initUART2(uint32_t baud_rate)
{
	uint32_t divisor, bus_clock;
	//Enable clock to UART2 and PORTE
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	//Connect UART pins for PTE22, PTE23, i.e. muxing to UART pin config
	//PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	//PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
	
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	//Ensure Tx and Rx are disabled before configuration
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	//Set Baud Rate to desired value
	bus_clock = (DEFAULT_SYSTEM_CLOCK) / 2; // Standard hardware bus clock config for UART, it runs half the system core clock
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	//No Parity, 8-bits
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	//Enable Tx and Rx
	//UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	UART2->C2 |= (UART_C2_RE_MASK);
	
	
	//Enable Interrupts for UART2
	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	//UART2->C2 |= (UART_C2_TIE_MASK | UART_C2_RIE_MASK);
	UART2->C2 |= (UART_C2_RIE_MASK);  // UART2->C2 |= ((UART_C2_TIE_MASK) | (UART_C2_RIE_MASK));
}

/* initMotor */
void initMotor(void) 
{
	// Enable Clock Gating for PORTA, PORTB, PORTD, PORTC and PORTE
	SIM_SCGC5 |= (SIM_SCGC5_PORTD_MASK);
	
	// Configure for the PWM pin operations
	PORTD->PCR[PTD0_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD0_PIN] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD1_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD1_PIN] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD2_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD2_PIN] |= PORT_PCR_MUX(4);
	
	PORTD->PCR[PTD3_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PTD3_PIN] |= PORT_PCR_MUX(4);
	
	// Enable Clock Gating for Timer0 and Timer1
	SIM->SCGC6 |= (SIM_SCGC6_TPM0_MASK);
	
	// Select Clock for TPM module
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // MCGFLLCLK or MCGPLLCLK/2  (internal clock of device is used), i.e. the Clock source
	
	// Set Modulo Value 20971520 / 128 = 163840 / 3276 = 50 Hz
	// TPM1->MOD = 3276;
	
	// Set Modulo Value 48000000 / 128 = 375000 / 7500 = 50 Hz
	TPM0->MOD = 7500;
	
	/* Edge-Alined PWM*/
	// Update SnC register: CMOD = 01, PS = 111 (128)
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	/* Channel Config, three modes for each channel*/
	// Enable PWM on TPM0 Channel 0 -> PTD0, i.e. channel config, set PTD0 pin to PWM mode and set to high-true pulse
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	// Enable PWM on TPM0 Channel 1 -> PTD1, i.e. channel config, set PTD1 pin to PWM mode and set to high-true pulse
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

}


void initUltrasonic(void) {
	
		// Enable clock gating for PORT E
		SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
		
		PORTE->PCR[PTE20_PIN] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[PTE20_PIN] |= PORT_PCR_MUX(3);   // T1C0 - Trig
	
		PORTE->PCR[PTE21_PIN] &= ~PORT_PCR_MUX_MASK;
		PORTE->PCR[PTE21_PIN] |= PORT_PCR_MUX(3);   // T1C1 - Echo
	
		// Enable Clock Gating for Timer1
		SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
  
		SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
		SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //MCGFLLCLK clock or MCGPLLCLK/2
		
		/* Edge-Alined PWM*/
		TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK) | (TPM_SC_CPWMS_MASK));
		TPM1->SC |= TPM_SC_PS(4); // Pre-scale factor: 16
	
		TPM1->MOD = 60000;
		
		// Enable interupt for echo pin, i.e. PTE21_PIN
		TPM1_C1SC |= TPM_CnSC_CHIE(1);
		
		TPM1_C0V = 30;
		
		NVIC_SetPriority(TPM1_IRQn, TPM1_INT_PRIO);
}


void initBuzzer(void) 
{
		SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
		PORTA->PCR[PTA1_PIN] &= ~PORT_PCR_MUX_MASK;
		PORTA->PCR[PTA1_PIN] |= PORT_PCR_MUX(3);
	
		SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
		SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
		SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
		// Set Modulo Value 48000000 / 128 = 375000 / 7500 = 50 Hz
		TPM2->MOD = 7500;
	
		// Edge aligned PWM 
		// Update SnC register: CMOD = 01, PS = 111 (128)
		TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
		TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
		TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
		// Enable PWM on TPM2 Channel 0 -> PTA1
		TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
		TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}


void UART2_IRQHandler(void) {
	
	myDataPkt sendData;
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		sendData.cmd = UART2->D;
		osMessageQueuePut(brainMsg,&sendData, NULL, 0);
	}
}

// Interrupt for ultrasonic sensor, input capture mode for PTE21_PIN
void TPM1_IRQHandler() {
		
		myDataPkt sendData;
	
		// Clear Pending IRQ
		NVIC_ClearPendingIRQ(TPM1_IRQn);
	
		// Clear channel interrupt flag
		TPM1_STATUS |= (TPM_STATUS_CH1F(1));
	
		if (risingEdge) {
			TPM1_CNT = 0;
			risingEdge = 0;
			
			// Configure Input Capture Mode on Timer 1 Channel 1, i.e. PTE21_PIN, to respond to falling edge
			TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
			TPM1_C1SC |= (TPM_CnSC_ELSB(1));
			
		} else {
			risingEdge = 1;
			sendData.dist = TPM1_C1V * 0.05715;
			NVIC_DisableIRQ(TPM1_IRQn);
			osMessageQueuePut(utReadingMsg, &sendData, NULL, 0);
		}
		
}

void offGreenLEDs(void) {
		
		PTC->PCOR |= MASK(PTC7_PIN);
		PTC->PCOR |= MASK(PTC0_PIN);
		PTC->PCOR |= MASK(PTC3_PIN);
		PTC->PCOR |= MASK(PTC4_PIN);
		PTC->PCOR |= MASK(PTC5_PIN);
		PTC->PCOR |= MASK(PTC6_PIN);
		PTC->PCOR |= MASK(PTC10_PIN);
		PTC->PCOR |= MASK(PTC11_PIN);		
}

void redLEDControl(led_state_t state) {
	if (state) {
		PTE->PSOR |= MASK(PTE5_PIN);
		PTE->PSOR |= MASK(PTE4_PIN);
		PTE->PSOR |= MASK(PTE3_PIN);
		PTE->PSOR |= MASK(PTE2_PIN);
		PTB->PSOR |= MASK(PTB11_PIN);
		PTB->PSOR |= MASK(PTB10_PIN);
		PTB->PSOR |= MASK(PTB9_PIN);
		PTB->PSOR |= MASK(PTB8_PIN);
	} else {
		PTE->PCOR |= MASK(PTE5_PIN);
		PTE->PCOR |= MASK(PTE4_PIN);
		PTE->PCOR |= MASK(PTE3_PIN);
		PTE->PCOR |= MASK(PTE2_PIN);				
		PTB->PCOR |= MASK(PTB11_PIN);
		PTB->PCOR |= MASK(PTB10_PIN);
		PTB->PCOR |= MASK(PTB9_PIN);
		PTB->PCOR |= MASK(PTB8_PIN);
	}
}


void greenLEDControl(int count, led_state_t state) {
		switch(count)
		{
			case 0:
				if (state)
					PTC->PSOR = MASK(PTC7_PIN);
				else 
					PTC->PCOR = MASK(PTC7_PIN);
				break;
			case 1:
				if (state)
					PTC->PSOR = MASK(PTC0_PIN);
				else 
					PTC->PCOR = MASK(PTC0_PIN);
				break;
			case 2:
				if (state)
					PTC->PSOR = MASK(PTC3_PIN);
				else 
					PTC->PCOR = MASK(PTC3_PIN);
				break;
			case 3:
				if (state)
					PTC->PSOR = MASK(PTC4_PIN);
				else
					PTC->PCOR = MASK(PTC4_PIN);
				break;
			case 4:
				if (state)
					PTC->PSOR = MASK(PTC5_PIN);
				else
					PTC->PCOR = MASK(PTC5_PIN);
				break;
			case 5:
				if (state) 
					PTC->PSOR = MASK(PTC6_PIN);
				else 
					PTC->PCOR = MASK(PTC6_PIN);
				break;
			case 6:
				if (state)
					PTC->PSOR = MASK(PTC10_PIN);
				else 
					PTC->PCOR = MASK(PTC10_PIN);
				break;
			case 7:
				if (state)
					PTC->PSOR = MASK(PTC11_PIN);
				else
					PTC->PCOR = MASK(PTC11_PIN);
				break;
			default:
				offGreenLEDs();
		}
}

void offLEDs(void) {
	offGreenLEDs();
	redLEDControl(led_off);
} 


void move(uint8_t movement) {
	
	/*
  (PTD0_PIN) TPM0_C0 - Top/Bottom left motor - Go Forward pin
  (PTD1_PIN) TPM0_C1 - Top/Bottom right motor - Go Forward Pin
  (PTD2_PIN) TPM0_C2 - Top/Bottom left motor - Go Back pin
  (PTD3_PIN) TPM0_C3 - Top/Bottom right motor - Go Back pin
	*/

	switch (movement) {
		case forward:
			TPM0_C0V = SPEED; 
			TPM0_C1V = SPEED;
			TPM0_C2V = 0x0;
			TPM0_C3V = 0x0;		
			break;
		
		case backward:
			TPM0_C0V = 0x0;    
			TPM0_C1V = 0x0;
			TPM0_C2V = SPEED;
			TPM0_C3V = SPEED;
			break;
		
		case left:
			TPM0_C0V = TURNSPEED;  
			TPM0_C1V = MAXSPEED;
			TPM0_C2V = 0x0;
			TPM0_C3V = 0x0;
			break;
		
		case right:
			TPM0_C0V = MAXSPEED;    
			TPM0_C1V = TURNSPEED;
			TPM0_C2V = 0x0;
			TPM0_C3V = 0x0;
			break;
		
		case stop:
			TPM0_C0V = 0x0;     
			TPM0_C1V = 0x0;
			TPM0_C2V = 0x0;
			TPM0_C3V = 0x0;
			break;
		
		case staticRight:
			TPM0_C0V = SPINSPEED;  
			TPM0_C1V = 0x0;
			TPM0_C2V = 0x0;
			TPM0_C3V = SPINSPEED;
			break;
		
		case staticLeft:
			TPM0_C0V = 0x0;    
			TPM0_C1V = SPINSPEED;
			TPM0_C2V = SPINSPEED;
			TPM0_C3V = 0x0;
			break;
		
		case backwardRight:
			TPM0_C0V = 0x0;    
			TPM0_C1V = 0x0;
			TPM0_C2V = MAXSPEED;
			TPM0_C3V = TURNSPEED;
			break;
		
		case backwardLeft:
			TPM0_C0V = 0x0;    
			TPM0_C1V = 0x0;
			TPM0_C2V = TURNSPEED;
			TPM0_C3V = MAXSPEED;
			break;
		
		default:
			TPM0_C0V = 0x0;     // Stop
			TPM0_C1V = 0x0;
			TPM0_C2V = 0x0;
			TPM0_C3V = 0x0;
	}
}


/*----------------------------------------------------------------------------
 * Application tUltrasonic thread
 *---------------------------------------------------------------------------*/
void tUltrasonic (void *argument) {

  for (;;) {
		osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
		
		//Stop timer
		TPM1_SC &= ~TPM_SC_CMOD_MASK;
		
		// Enable Output Compare Mode on Timer 1 Channel 0, PTE20_PIN, to generate 10 microsec high pulse when timer starts
		TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
		TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSA(1));
		
		// Configure Input Capture Mode on Timer 1 Channel 1, PTE21_PIN, to respond to rising edge
		TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM1_C1SC |= TPM_CnSC_ELSA(1);
		
		// Reset Timer count value to 0
		TPM1_CNT = 0;
		
		risingEdge = 1;
		NVIC_EnableIRQ(TPM1_IRQn);
		NVIC_ClearPendingIRQ(TPM1_IRQn);
		
		// Start Timer
    TPM1_SC |= TPM_SC_CMOD(1);
		
		osDelay(50);
		
	}
}

/*----------------------------------------------------------------------------
 * Application tAutoDrive thread
 *---------------------------------------------------------------------------*/
void tAutoDrive (void *argument) {
 
  myDataPkt myRxData, utMeasure; 
	myDataPkt motorData;
	osStatus_t fxCheck;
  for (;;) {
		osStatus_t messageStatus = osMessageQueueGet(autoDriveMsg, &myRxData, NULL, osWaitForever);	
		if (MESSAGE_MASK(myRxData.cmd) == start) {
			
			
			int distanceReading = 1000;
			motorData.cmd = forward;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			while (distanceReading > 380) {
				osThreadFlagsSet(ultrasonic_Id, 0x0001);
				osDelay(60);
				
				fxCheck = osMessageQueueGet(utReadingMsg, &utMeasure, NULL, 70);
				if (fxCheck == osOK) {
					distanceReading = utMeasure.dist;
				}  
			}
			motorData.cmd = backward;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(200);
			motorData.cmd = stop;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			motorData.cmd = staticLeft;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(300);
			motorData.cmd = stop;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			motorData.cmd = forward;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(400);
			motorData.cmd = stop;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(100);
			motorData.cmd = staticRight;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(500);
			motorData.cmd = stop;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			motorData.cmd = forward;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(400);
			motorData.cmd = stop;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			motorData.cmd = staticRight;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(550);
			motorData.cmd = stop;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			motorData.cmd = forward;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(480);
			motorData.cmd = stop;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			motorData.cmd = staticRight;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(500);
			motorData.cmd = stop;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			motorData.cmd = forward;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(530);
			motorData.cmd = stop;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			motorData.cmd = staticLeft;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(310);
			motorData.cmd = stop;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			motorData.cmd = forward;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			distanceReading = 1000;
			while (distanceReading > 300) {
				osThreadFlagsSet(ultrasonic_Id, 0x0001);
				osDelay(60);
				
				fxCheck = osMessageQueueGet(utReadingMsg, &utMeasure, NULL, 70);
				if (fxCheck == osOK) {
					distanceReading = utMeasure.dist;
				}  
			}
			move(backward);
			osDelay(200);
			motorData.cmd = stopRun;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
			osDelay(50);
			
		}
		if (MESSAGE_MASK(myRxData.cmd) == end) {
			motorData.cmd = stopRun;
			osMessageQueuePut(motorMsg, &motorData, NULL, 0);
		}
	}
}

/*----------------------------------------------------------------------------
 * Application tMotorControl thread
 *---------------------------------------------------------------------------*/
void tMotorControl (void *argument) {
  
	myDataPkt myRxData;
	myDataPkt ledData, audioData;
  for (;;) {
		osMessageQueueGet(motorMsg, &myRxData, NULL, osWaitForever);
		if (MESSAGE_MASK(myRxData.cmd) == stop) {
			ledData.cmd = stopped;
			osMessageQueuePut(ledMsg, &ledData, NULL, 0);
			move(MESSAGE_MASK(myRxData.cmd));
			
		} else if (MESSAGE_MASK(myRxData.cmd) == stopRun) {
			ledData.cmd = stopped;
			audioData.cmd = unique_tune;
			osMessageQueuePut(ledMsg, &ledData, NULL, 0);
			osMessageQueuePut(audioMsg, &audioData, NULL, 0);
			move(MESSAGE_MASK(myRxData.cmd));
			
		} else {
			ledData.cmd = moving;
			audioData.cmd = challenge_tune;
			osMessageQueuePut(ledMsg, &ledData, NULL, 0);
			osMessageQueuePut(audioMsg, &audioData, NULL, 0);
			move(MESSAGE_MASK(myRxData.cmd));
		}
		
	}
}

/*----------------------------------------------------------------------------
 * Application tAudio thread
 *---------------------------------------------------------------------------*/
void tAudio (void *argument) {
 
	myDataPkt myRxData;
	osStatus_t fxCheck;
	int run = 0;
	for(;;) {
		fxCheck = osMessageQueueGet(audioMsg, &myRxData, NULL, 0);
		if (fxCheck == osOK) {		
			if (MESSAGE_MASK(myRxData.cmd) == blank) {
				TPM2_C0V = 0x0;
			} else if (MESSAGE_MASK(myRxData.cmd) == unique_tune) {
				for(char i = 0; i < TONE_CNT; i++) {
					TPM2->MOD = FREQ_2_MOD(uniqueTone[i]);
					TPM2_C0V = (FREQ_2_MOD(uniqueTone[i])) / 2;
					osDelay(500);
				}
				TPM2_C0V = 0x0;
			} else if (MESSAGE_MASK(myRxData.cmd) == challenge_tune) {
				for(int i = 0; i < SONG_CNT; i++) {
					TPM2->MOD = FREQ_2_MOD(song[i]);
					TPM2_C0V = (FREQ_2_MOD(song[i])) / 2;
					osDelay(600);
				}
				TPM2_C0V = 0x0;
				run = 1;
			}
		} else {
			if (run) {
				for(int i = 0; i < SONG_CNT; i++) {
					TPM2->MOD = FREQ_2_MOD(song[i]);
					TPM2_C0V = (FREQ_2_MOD(song[i])) / 2;
					osDelay(600);
				}
				TPM2_C0V = 0x0;				
			} else {
				TPM2_C0V = 0x0;
			}
		}
	}
}

/*----------------------------------------------------------------------------
 * Application tLED thread
 *---------------------------------------------------------------------------*/
void tLED (void *argument) {
 
  myDataPkt myRxData;
	osStatus_t dataCheck;
	int i = -1;
	int flag = 0;
	offLEDs();
  for (;;) {
		osMessageQueueGet(ledMsg, &myRxData, NULL, 100);
		if (dataCheck == osOK) {
			if (MESSAGE_MASK(myRxData.cmd) == nothing) {
				offLEDs();
			} else if (MESSAGE_MASK(myRxData.cmd) == moving) {
				offLEDs();
				greenLEDControl(i, led_off);
				i = (i < 7)? i + 1: 0;
				greenLEDControl(i, led_on);
				redLEDControl(led_on);
				osDelay(500);
				greenLEDControl(i, led_off);
				i = (i < 7)? i + 1: 0;
				greenLEDControl(i, led_on);
				redLEDControl(led_off);
				osDelay(500);
				flag = 1;
				
			} else if (MESSAGE_MASK(myRxData.cmd) == wifi_check) {	
				offGreenLEDs();
				PTC->PSOR |= MASK(PTC7_PIN);
				PTC->PSOR |= MASK(PTC11_PIN);
				
			} else {
				offLEDs();
				for (int j = 0; j < 8; j++) {
					greenLEDControl(j, led_on);
				}
				redLEDControl(led_on);
				osDelay(250);
				redLEDControl(led_off);
				osDelay(250);
			}
		} else {
			if (flag) {
				offLEDs();
				for (int j = 0; j < 8; j++) {
					greenLEDControl(j, led_on);
				}
				redLEDControl(led_on);
				osDelay(250);
				redLEDControl(led_off);
				osDelay(250); 
			} else {
				offLEDs();
			}
		}
		
	}
		
}

/*----------------------------------------------------------------------------
 * Application tBrain thread
 *---------------------------------------------------------------------------*/
void tBrain (void *argument) {
 
	myDataPkt myData;
	myDataPkt sendData;
  for (;;) {
		osMessageQueueGet(brainMsg, &myData, NULL, osWaitForever);
		if (FUNCTION_MASK(myData.cmd) == MOTOR_FUNCTION) {
			osMessageQueuePut(motorMsg, &myData, NULL, 0);
		} 
		if (FUNCTION_MASK(myData.cmd) == WIFI_FUNCTION) { 
			sendData.cmd = wifi_check;
			osMessageQueuePut(audioMsg, &sendData, NULL, 0);
			osMessageQueuePut(ledMsg, &sendData, NULL, 0);
			osDelay(1000);
		} 
				
		if (FUNCTION_MASK(myData.cmd) == AUTO_DRIVE_FUNCTION) {
			osMessageQueuePut(autoDriveMsg, &myData, NULL, 0);
		}
	}
}

int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initLEDs();
	initUART2(BAUD_RATE);
	initMotor();
	initBuzzer();
	initUltrasonic();
	offLEDs();

		
	
	
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	
	motor_Id = osThreadNew(tMotorControl, NULL, NULL);
	audio_Id = osThreadNew(tAudio, NULL, NULL);
	led_Id = osThreadNew(tLED, NULL, NULL);
	autoDrive_Id = osThreadNew(tAutoDrive, NULL, NULL);
	ultrasonic_Id = osThreadNew(tUltrasonic, NULL, NULL);
	brain_Id = osThreadNew(tBrain, NULL, &brainPriority);
	
	motorMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	audioMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	ledMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	autoDriveMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	utMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	brainMsg = osMessageQueueNew(8, sizeof(myDataPkt), NULL);
	utReadingMsg = osMessageQueueNew(MSG_COUNT, sizeof(myDataPkt), NULL);
	
  osKernelStart();                      // Start thread execution
  for (;;) {}
	
}
