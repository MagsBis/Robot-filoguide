#include "LPC17xx.h"
int MESSAGE_READY = 0;
int mask = 0;
typedef struct {
	int TYPE;
	int NUM;
	int VIT;
	int LETTRE;
	int NUM_POSTE;
}COMMANDE_ROBOT;
int SETUP_MESSAGE = 0x00000128;
int MESSAGE = 0;
int bit = 0;
COMMANDE_ROBOT com = {0,0,0,0,0};
int bit_counter = 13;
void init_PINS_BASE() {
	LPC_PINCON->PINSEL0 = 0x0000005A;
	LPC_PINCON->PINSEL3 = 0x00000020;
	
	LPC_GPIO0->FIODIR = 0x00000015;
	LPC_GPIO1->FIODIR = 0x00040000;
	LPC_GPIO2->FIODIR = 0x00000000;
	
	LPC_PINCON->PINMODE0 =0x00000300 ;
	LPC_PINCON->PINMODE3 =0xFF000000 ;
	LPC_PINCON->PINMODE4 =0x003FF000 ;
	
}

void CODE_FIL() {
	switch(com.TYPE) { 
		case 0: MESSAGE = (com.TYPE << 11) + (com.NUM << 7) + com.VIT;
						break;
		case 1: MESSAGE = (com.TYPE << 11) + (com.NUM << 7) + (com.LETTRE << 5) + (com.NUM_POSTE);
						break;
		case 2: MESSAGE = (com.TYPE << 11) + (com.NUM << 7) + (com.LETTRE << 5) + (com.NUM_POSTE);

						break;
		case 3: MESSAGE = (com.TYPE << 11) + (com.NUM << 7) + (com.LETTRE << 5) + (com.NUM_POSTE);

						break;
		case 4: MESSAGE = (com.TYPE << 11) + (com.NUM << 7) + (com.LETTRE << 5) + (com.NUM_POSTE);

						break;
		// mode debug à ajouter après
		default : return;
	}
}

void init_TIMER0() {
	
	LPC_TIM0->MR0 = 25E6*3E-3;
	LPC_TIM0->MR1 = 25E6*25E-4;
	LPC_TIM0->MCR = 3 | (1 << 3);
	LPC_TIM0->IR |= 3;
	NVIC_EnableIRQ(TIMER0_IRQn);
	LPC_TIM0->TCR |= 1;
}
void init_EMISSION_FIL() {
	LPC_PWM1->MR0 = 25E6*1/ 50E3;
	LPC_PWM1->MR1 = 25E6*1/ 100E3;
	LPC_PWM1->TCR |= 1 << 3;
	LPC_PWM1->TCR |= 1;
	LPC_PWM1->MCR |= 2;
	LPC_PWM1->PCR |= 1 << 9;
	LPC_PWM1->LER |= 3;
}

void TIMER0_IRQHandler() {
	if(LPC_TIM0->IR == 1) {
		LPC_PWM1->TCR |= 1;
	}
	else {
		LPC_PWM1->TCR &= ~1;
		LPC_PWM1->TCR |= 2;
		LPC_PWM1->TCR &= ~2;
	}
	if ((MESSAGE_READY == 1) && (bit_counter > -1) && (LPC_TIM0->IR  == 1)) {
		LPC_TIM0->IR |= 3;
		mask = (MESSAGE & (1 << bit_counter)) >> bit_counter; 
		switch((MESSAGE & (1 << bit_counter)) >> bit_counter) {
			case 1: LPC_TIM0->MR1 = 25E6*175E-5;
							LPC_TIM0->MR0 = 25E6*225E-5;
							bit_counter--;
							break;
							
			case 0: LPC_TIM0->MR1 = 25E6*1E-3;
							LPC_TIM0->MR0 = 25E6*15E-4;
							bit_counter--;
							break;
			default: return;
			
		}
		
	}
	else if ((MESSAGE_READY == 1) && (bit_counter > -1) && (LPC_TIM0->IR  != 1)) {
		LPC_TIM0->IR |= 3;

	}
	else {
		LPC_TIM0->IR |= 3;
		LPC_TIM0->MR0 = 25E6*3E-3;
		LPC_TIM0->MR1 = 25E6*25E-4;
		MESSAGE_READY = 0;
		bit_counter = 13;
	}
	
}

int main() {
	init_PINS_BASE();
	init_TIMER0();
	init_EMISSION_FIL();
	com.NUM = 3;
	com.TYPE = 1;
	com.NUM_POSTE = 13;
	com.LETTRE = 1;
	MESSAGE = SETUP_MESSAGE;
	MESSAGE_READY = 1;
	while(1) {

	}
	
	return 0;
	
}