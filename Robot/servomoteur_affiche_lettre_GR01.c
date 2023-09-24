#include "LPC17xx.h"

unsigned int ordre=(1<<12);//|(1<<5); //(à enlever lors de la fusion des codes) en fonction de l'ordre décoder par le capteur à induction par la bobine verticale
unsigned int status;


void init_pwm()
{
	LPC_PINCON->PINSEL4 |= 1<<4;  //on utilse le P2.3 en pwm
	LPC_PINCON->PINSEL4 &= ~(1<<5);
	LPC_PWM1->PCR &= ~(1<<3);  //single edge
	LPC_PWM1->PCR |= 1<<11;   //activation PWM3
	LPC_PWM1->MR0 = 500000; //25MHz*20ms 
	LPC_PWM1->MR3 = 0; //on tourne pas le servomoteur au début
	LPC_PWM1->LER |= (1<<3)|1;
	LPC_PWM1->MCR |= 3<<9;    
	NVIC_EnableIRQ(PWM1_IRQn); 
}



void aff_lettre()
{
	if ((ordre & (1<<12)) | (ordre & (1<<13)) ) //si on reçoit un ordre de chargement à droite ou à gauche cela voudra dire qu'on a affecté au robot une lettre
	{
		if (!(ordre&(3<<5))) //si LL='00' on veut lettre A
		{
			LPC_PWM1->MR3 = 35000; // il y a 5 angle possible on va diviser le cercle en 5 pour chaque lettre+lettre vide (360/5=72degré) donc pour chaque changement de lettre on tourne d'un angle 72degré la lettre vide sera à 0degré donc A sera à 72degré 0degré=1ms et un écart de 90degré=>0.5ms avec prod en croix 72degré=>0.4ms, 1ms+0.4ms=1.4ms le pwm devra être pendant 1.4ms à '1' et 25MHz*1.4ms=35000
			LPC_PWM1->LER |= (1<<3);
			LPC_PWM1->TCR |= (1<<3)|1; //si on affecte une lettre au robot on lance le pwm
		}
		else if ((ordre&(3<<5))==(1<<5)) //si LL='01' on veut lettre B
		{
			LPC_PWM1->MR3 = 45000; // lettre B à 144degré=>0.8ms 1ms+0.8ms=1.8ms 25MHz*1.8ms=45000 
			LPC_PWM1->LER |= (1<<3)|1;
			LPC_PWM1->TCR |= (1<<3)|1; //si on affecte une lettre au robot on lance le pwm
		}
		else if ((ordre&(3<<5))==(1<<6)) //si LL='10' on veut lettre C
		{
			LPC_PWM1->MR3 = 55000; // lettre C à 216degré=>1.2ms 1ms+1.2ms=2.2ms 25MHz*2.2ms=55000 
			LPC_PWM1->LER |= (1<<3);
			LPC_PWM1->TCR |= (1<<3)|1; //si on affecte une lettre au robot on lance le pwm
		}
		else //si LL='11' on veut lettre D
		{
			LPC_PWM1->MR3 = 65000; // lettre B à 288degré=>1.6ms 1ms+1.6ms=2.6ms 25MHz*2.6ms=65000 
			LPC_PWM1->LER |= (1<<3);
			LPC_PWM1->TCR |= (1<<3)|1; //si on affecte une lettre au robot on lance le pwm
		}
	}
	else if (status==1) //status dispo 
	{
		LPC_PWM1->MR3 = 25000; // lettre ___ à 0degré=>1ms
		LPC_PWM1->LER |= (1<<3);
		LPC_PWM1->TCR |= (1<<3)|1; 
	}
}




void PWM1_IRQHandler()
{
	LPC_PWM1->IR |=1;
	LPC_PWM1->TCR &= ~1; //après l'impulsion on aura montré la lettre désirée on peut donc arreter le pwm
}

int main()
{
	aff_lettre();
	init_pwm();
	while(1)
		{

		}
	
}