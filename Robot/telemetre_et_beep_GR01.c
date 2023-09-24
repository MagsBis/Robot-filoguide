#include "LPC17xx.h"

unsigned int nb_mes; //variable qui indique la valeur décimal du nombre de mesure à faire
unsigned int nb_mes_etat; //variable qui indique la valeur binaire du nombre de mesure à faire
float temps_mes;  //le temps(en nombre d'incrémentation du Prescale Register) qu'il faut pour faire le nombre de mesure nécessaire en 1 seconde
float temps; //le temps qui sera en seconde
int t0,t1;  //t0 qui garde la valeur de l'envoi d'une impulsion t1 le retour de l'impulsion
int mes_effectue=0; //le nombre de mesure qui a été effectué
float distance; //en cm
unsigned int vitesse = 20;
unsigned int status = 3;
unsigned int nb_beep_ouvrier;

void init_gpio()
{
  LPC_PINCON->PINSEL2 &= ~(3);
  LPC_GPIO1->FIODIR |= 1;
  LPC_GPIO1->FIOPIN &= ~1; //ecriture signal emission_ON
  LPC_PINCON->PINSEL1 &= ~(3<<20);
  LPC_GPIO0->FIODIR &= ~(1<<26); //lecture signal de reception TOP_SIG_RECU
  LPC_GPIO0->FIOPIN &= ~(1<<26);
  LPC_PINCON->PINSEL0 &= (15<<4);
  LPC_GPIO0->FIODIR &= ~(3<<2); //lecture nb_mes_0 et nb_mes_1    
}

void init_T1()
{
	LPC_TIM1->MR0 = 625/2;
	LPC_TIM1->MCR |= 3; 
	LPC_TIM1->MCR &=~(1<<2);
	NVIC_EnableIRQ(TIMER1_IRQn);
}


void init_T3()
{
	LPC_TIM3->MR1 = 0;//MR pour le moment d'envoi d'impulsion
	LPC_TIM3->MR2 = 25000000*temps_mes/10; //MR pour la durée d'envoi de l'impulsion
	LPC_TIM3->MCR |= (1<<6)|(1<<3); //on veut juste une interruption pas de reset ni stop car on va incrémenter les valeurs de MR1 et MR2
	LPC_TIM3->MCR &=~(1<<7);
	LPC_TIM3->MCR &=~(1<<4);
	NVIC_EnableIRQ(TIMER3_IRQn);
	LPC_TIM3->TCR |= 1;
}

void TIMER1_IRQHandler()
{
	LPC_TIM1->IR |= 1;
	LPC_GPIO1->FIOPIN ^= 1;
}

void TIMER3_IRQHandler()
{
	if (nb_mes_etat != ((LPC_GPIO0->FIOPIN & (3<<2)) >>2)) //si en cours de temps on à changer le nombre de mesure voulut avec les interrupteur nb_mes_o et nb_mes_1 on doit mettre à jour nb_mes_etat et nb_mes et donc aussi temps_mes
   	 {
   		 nb_mes_etat = ((LPC_GPIO0->FIOPIN & (3<<2)) >>2);  //mise à jour de nb_mes_etat
   		 if (!nb_mes_etat) //cas '00'
   		 {
   				 nb_mes=25;
   				 temps_mes=1./nb_mes;	//mise à jour de temps_mes car nb_mes a changé
   				 LPC_TIM3->MR2 += 25000000*temps_mes/10;
   		 }
   		 else if (nb_mes_etat==1) //cas '01'
   		 {
   				 nb_mes=20;
   				 temps_mes=1./nb_mes;
   				 LPC_TIM3->MR2 += 25000000*temps_mes/10;
   		 }
   		 else if (nb_mes_etat==2) //cas '10'
   		 {
   				 nb_mes=15;
   				 temps_mes=1./nb_mes;
   				 LPC_TIM3->MR2 +=  25000000*temps_mes/10;
   		 }
   		 else if (nb_mes_etat==3) //cas '11'
   		 {
   				 nb_mes=10;
   				 temps_mes=1./nb_mes;
   				 LPC_TIM3->MR2 += 25000000*temps_mes/10;
   		 }
   	 }
	if (LPC_TIM3->IR&(1<<1))  //si l'interruption a été causer par le MR1
	{
    	LPC_TIM3->IR |= 1<<1;
    	mes_effectue++;  //un nouveau signal a été envoyer donc on incrémente le nombre de mesure faite
    	LPC_TIM3->MR1 += 25000000*temps_mes; //incrémentation pour le prochain envoi
    	LPC_GPIO1->FIOPIN |=1;  //émission de l'imuplsion
			LPC_TIM1->TCR |= 1;
			LPC_TIM1->TCR &= ~(1<<1);
    	t0=LPC_TIM3->TC; //mesure temps du premier envoi
	}
    
	if (LPC_TIM3->IR&(1<<2))  //si l'interruption a été causer par le MR2
	{
    	LPC_TIM3->IR |= 1<<2;
			LPC_TIM1->TCR &= ~1;
			LPC_TIM1->TCR |= (1<<1);
    	LPC_GPIO1->FIOPIN &= ~1; //fin d'émission de l'imuplsion
    	LPC_TIM3->MR2 += 25000000*temps_mes; //incrémentation pour le prochain envoi
	}
    
	if (mes_effectue==nb_mes)   //si on a fait toute le nombre de mesures désiré on remet les MR à leur valeurs initiales pour éviter des overflow on fait donc aussi un reset sur le TCR
	{
    	LPC_TIM3->MR1 = 0;
   		LPC_TIM3->MR2 = 25000000*temps_mes/10;
    	LPC_TIM3->TCR = 2;
    	LPC_TIM3->TCR = 1;
    	LPC_GPIO1->FIOPIN ^= 1;
    	mes_effectue=-1;
	}
}


void sig_recue()
{
	LPC_GPIOINT->IO0IntEnR |= 1<<26;
	LPC_GPIOINT->IO0IntClr |= 1<<26;
	NVIC_EnableIRQ(EINT3_IRQn);
}


void beep()
{
	LPC_PINCON->PINSEL2 &= ~(3<<28);
	LPC_GPIO1->FIODIR |= 1<<14;
	LPC_GPIO1->FIOPIN &= ~(1<<14);
	LPC_TIM2->MR0 = 0;
	LPC_TIM2->MCR |= (1<<6)|(1<<3)|(1<<1)|1;
	LPC_TIM2->MCR &= ~(1<<2);
	NVIC_EnableIRQ(TIMER2_IRQn);
}

void TIMER2_IRQHandler()
{
	if (vitesse) //si le robot est entrain de rouler on envoi des beep qui dépendent de la proximité des obstacles
	{
		if ((LPC_TIM2->IR) & (1)) //si interruption sur MR0 on commence l'envoi du beep
		{
			nb_beep_ouvrier=0;
			LPC_TIM2->IR |= 1;
			LPC_GPIO1->FIOPIN |= (1<<14);
			if (distance>=100)
			{
				LPC_TIM2->MR0 = 25000000/0.5;
				LPC_TIM2->MR1 = 25000000*0.25;
				LPC_TIM2->MR2 = 25000/2;
			}
			else if (distance<100 && distance>=90)
			{
				LPC_TIM2->MR0 = 25000000/0.75;
				LPC_TIM2->MR1 = 25000000*0.25;
				LPC_TIM2->MR2 = 25000/2;
			}
			else if (distance<90 && distance>=80)
			{
				LPC_TIM2->MR0 = 25000000;
				LPC_TIM2->MR1 = 25000000*0.25;
				LPC_TIM2->MR2 = 25000/2;
			}
			else if (distance<80 && distance>=70)
			{
				LPC_TIM2->MR0 = 25000000/1.25;
				LPC_TIM2->MR1 = 25000000*0.25;
				LPC_TIM2->MR2 = 25000/2;
			}
			else if (distance<70 && distance>=60)
			{
				LPC_TIM2->MR0 = 25000000/1.5;
				LPC_TIM2->MR1 = 25000000*0.25;
				LPC_TIM2->MR2 = 25000/2;
			}
			else if (distance<60 && distance>=50)
			{
				LPC_TIM2->MR0 = 25000000/1.75;
				LPC_TIM2->MR1 = 25000000*0.25;
				LPC_TIM2->MR2 = 25000/2;
			}
			else if (distance<50 && distance>=40)
			{
				LPC_TIM2->MR0 = 25000000/2;
				LPC_TIM2->MR1 = 25000000*0.25;
				LPC_TIM2->MR2 = 25000/2;
			}
			else if (distance<40 && distance>=30)
			{
				LPC_TIM2->MR0 = 25000000/2.25;
				LPC_TIM2->MR1 = 25000000*0.25;
				LPC_TIM2->MR2 = 25000/2;
			}
			else if (distance<30 && distance>=20)
			{
				LPC_TIM2->MR0 = 25000000/2.5;
				LPC_TIM2->MR1 = 25000000*0.25;
				LPC_TIM2->MR2 = 25000/2;
			}
		}
		if ((LPC_TIM2->IR) & (1<<1)) //si interruption sur MR1 on stop l'envoi du beep
		{
			LPC_TIM2->IR |= 1<<1;
			LPC_GPIO1->FIOPIN &= ~(1<<14);
		}
		if ((LPC_TIM2->IR) & (1<<2)) //si interruption sur MR2 on réalise la fréquence de 1kHz du beep
		{
			LPC_TIM2->IR |= 1<<2;
			LPC_GPIO1->FIOPIN ^= (1<<14);
			LPC_TIM2->MR2 += 25000/2;
		}
	}
	else //si le robot s'arrête
	{
		if (status==2) //si status enlevecolis l'ouvrier doit charger donc 2 beeps
		{
			if (nb_beep_ouvrier<=2)
			{
				if ((LPC_TIM2->IR) & (1))
				{
					LPC_TIM2->IR |= 1;
					LPC_TIM2->MR0 = 25000000*1.25; //durée jusqu'au prochain envoi de beep
					LPC_TIM2->MR1 = 25000000*0.75; //durée d'un beep
					LPC_TIM2->MR2 = 25000/2; //fréquence 1kHz du beep
					nb_beep_ouvrier++;
				}
				if ((LPC_TIM2->IR) & (1<<1))
				{
					LPC_TIM2->IR |= 1<<1;
					LPC_GPIO1->FIOPIN &= ~(1<<14);
				}
				if ((LPC_TIM2->IR) & (1<<2)) //si interruption sur MR2 on réalise la fréquence de 1kHz du beep
				{
					LPC_TIM2->IR |= 1<<2;
					LPC_GPIO1->FIOPIN ^= (1<<14);
					LPC_TIM2->MR2 += 25000/2;
				}
			}
		}
		else if (status==3)
		{
			if (nb_beep_ouvrier<=1)
			{
				if ((LPC_TIM2->IR) & (1))
				{
					LPC_TIM2->IR |= 1;
					LPC_TIM2->MR0 = 25000000*1.25;
					LPC_TIM2->MR1 = 25000000*0.75; //durée d'un beep
					LPC_TIM2->MR2 = 25000/2; //fréquence 1kHz du beep
					nb_beep_ouvrier++;
				}
				if ((LPC_TIM2->IR) & (1<<1))
				{
					LPC_TIM2->IR |= 1<<1;
					LPC_GPIO1->FIOPIN &= ~(1<<14);
				}
				if ((LPC_TIM2->IR) & (1<<2)) //si interruption sur MR2 on réalise la fréquence de 1kHz du beep
				{
					LPC_TIM2->IR |= 1<<2;
					LPC_GPIO1->FIOPIN ^= (1<<14);
					LPC_TIM2->MR2 += 25000/2;
				}
			}
		}
	}
}

void EINT3_IRQHandler()
{
	LPC_GPIOINT->IO0IntClr |= 1<<26;
	t1=LPC_TIM3->TC;
	temps=(t1-t0)*1/25000000./2; //le TC nous donne le nombre de fois que le Prescal Register s'est incrémenté et on multiplie par le temps du PCLK=25MHz et on /2 car on veut pas le temps de l'aller retour mais juste un aller
  distance = 340*temps*100; //340m/s vitesse son dans air *100pour convertir m en cm
	NVIC_EnableIRQ(TIMER2_IRQn);
	LPC_TIM2->TCR |= 1;
}

int main()
{
	init_gpio();
	init_T1();
	beep();
	init_T3();
	sig_recue();
	while(1); 
	return 0;
}



