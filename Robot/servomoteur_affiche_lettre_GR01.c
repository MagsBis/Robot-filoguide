#include "LPC17xx.h"

unsigned int ordre=(1<<12);//|(1<<5); //(� enlever lors de la fusion des codes) en fonction de l'ordre d�coder par le capteur � induction par la bobine verticale
unsigned int status;


void init_pwm()
{
	LPC_PINCON->PINSEL4 |= 1<<4;  //on utilse le P2.3 en pwm
	LPC_PINCON->PINSEL4 &= ~(1<<5);
	LPC_PWM1->PCR &= ~(1<<3);  //single edge
	LPC_PWM1->PCR |= 1<<11;   //activation PWM3
	LPC_PWM1->MR0 = 500000; //25MHz*20ms 
	LPC_PWM1->MR3 = 0; //on tourne pas le servomoteur au d�but
	LPC_PWM1->LER |= (1<<3)|1;
	LPC_PWM1->MCR |= 3<<9;    
	NVIC_EnableIRQ(PWM1_IRQn); 
}



void aff_lettre()
{
	if ((ordre & (1<<12)) | (ordre & (1<<13)) ) //si on re�oit un ordre de chargement � droite ou � gauche cela voudra dire qu'on a affect� au robot une lettre
	{
		if (!(ordre&(3<<5))) //si LL='00' on veut lettre A
		{
			LPC_PWM1->MR3 = 35000; // il y a 5 angle possible on va diviser le cercle en 5 pour chaque lettre+lettre vide (360/5=72degr�) donc pour chaque changement de lettre on tourne d'un angle 72degr� la lettre vide sera � 0degr� donc A sera � 72degr� 0degr�=1ms et un �cart de 90degr�=>0.5ms avec prod en croix 72degr�=>0.4ms, 1ms+0.4ms=1.4ms le pwm devra �tre pendant 1.4ms � '1' et 25MHz*1.4ms=35000
			LPC_PWM1->LER |= (1<<3);
			LPC_PWM1->TCR |= (1<<3)|1; //si on affecte une lettre au robot on lance le pwm
		}
		else if ((ordre&(3<<5))==(1<<5)) //si LL='01' on veut lettre B
		{
			LPC_PWM1->MR3 = 45000; // lettre B � 144degr�=>0.8ms 1ms+0.8ms=1.8ms 25MHz*1.8ms=45000 
			LPC_PWM1->LER |= (1<<3)|1;
			LPC_PWM1->TCR |= (1<<3)|1; //si on affecte une lettre au robot on lance le pwm
		}
		else if ((ordre&(3<<5))==(1<<6)) //si LL='10' on veut lettre C
		{
			LPC_PWM1->MR3 = 55000; // lettre C � 216degr�=>1.2ms 1ms+1.2ms=2.2ms 25MHz*2.2ms=55000 
			LPC_PWM1->LER |= (1<<3);
			LPC_PWM1->TCR |= (1<<3)|1; //si on affecte une lettre au robot on lance le pwm
		}
		else //si LL='11' on veut lettre D
		{
			LPC_PWM1->MR3 = 65000; // lettre B � 288degr�=>1.6ms 1ms+1.6ms=2.6ms 25MHz*2.6ms=65000 
			LPC_PWM1->LER |= (1<<3);
			LPC_PWM1->TCR |= (1<<3)|1; //si on affecte une lettre au robot on lance le pwm
		}
	}
	else if (status==1) //status dispo 
	{
		LPC_PWM1->MR3 = 25000; // lettre ___ � 0degr�=>1ms
		LPC_PWM1->LER |= (1<<3);
		LPC_PWM1->TCR |= (1<<3)|1; 
	}
}




void PWM1_IRQHandler()
{
	LPC_PWM1->IR |=1;
	LPC_PWM1->TCR &= ~1; //apr�s l'impulsion on aura montr� la lettre d�sir�e on peut donc arreter le pwm
}

int main()
{
	aff_lettre();
	init_pwm();
	while(1)
		{

		}
	
}