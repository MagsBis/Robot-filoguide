#include "LPC17xx.h"

int alim = 0,rep,oldbp[16],bp[16],commande;
int appui[16];
char info[16] = {'D','C','B','A','#','9','6','3','0','8','5','2','*','7','4','1'};
int i = 0,a;
void init_GPIO_clavier()
{
	LPC_PINCON->PINSEL0 &= ~(0xF<<3);
	LPC_GPIO0->FIODIR |= 0x3<<3;
	LPC_PINCON->PINSEL1 &= ~(0xFF);
	LPC_GPIO0->FIODIR &= ~(0xF<<16);
	LPC_GPIO0->FIODIR |= (0xF<<20);
}
void init_interruption_clavier()
{
	LPC_GPIOINT->IO0IntEnF |= (0xF<<16);   //interruption interne pr GPIO; interruption externe : LPC_SC->
	NVIC_EnableIRQ(EINT3_IRQn);
}
void delay()
{
		int i,j=0;
		for (i = 0; i < 25000000/10/2; i++){j++;}
}
void init_T0_clavier()
{
	int i;
	//LPC_SC->PCONP |= 1<<22; //pour timer 2;
	for (i = 0; i < 16; i++){
		bp[i] = 1;
	}
	LPC_TIM0->MR0 = 25000000/100;
	LPC_TIM0->MCR = 3;
	NVIC_EnableIRQ(TIMER0_IRQn);
	LPC_TIM0->TCR = 1;
}

void TIMER0_IRQHandler()
{
	LPC_TIM0->IR = 1;
	static int i = 0;
	LPC_GPIO0->FIOPIN |= (0xF<<20);
	LPC_GPIO0->FIOPIN &= ~(1<<(i+20));
	i++;
	if(i == 4) i = 0;
}
void init_T1_clavier()
{
  //LPC_SC->PCONP |= 1<<23; //pour timer 3;  
	LPC_TIM1->MR1 = 0;
  LPC_TIM1->MR2 = 25000000/10/3;
  LPC_TIM1->MCR |= (1<<6)|(1<<3);
  LPC_TIM1->MCR &=~(1<<7);
  LPC_TIM1->MCR &=~(1<<4);
  NVIC_EnableIRQ(TIMER1_IRQn);
  LPC_TIM1->TCR &=~1;
}
void TIMER1_IRQHandler()
{
	LPC_TIM1->IR = 1;
	i++;
	a = commande & 1<<(8-i/2);
	a = a >> (8-i/2);
	LPC_GPIO0->FIOPIN = a<<3;
	if (LPC_TIM1->IR&(1<<1)){
      LPC_TIM1->IR |= 1<<1;
      LPC_TIM1->MR1 += 25000000/10; 
      LPC_GPIO0->FIOPIN |=1<<4;
		
  }
  if (LPC_TIM1->IR&(1<<2)){
      LPC_TIM1->IR |= 1<<2;
      LPC_GPIO0->FIOPIN &= ~(1<<4);
      LPC_TIM1->MR2 += 25000000/10;
  }
	if(i % 16 == 0){
		LPC_TIM1->TCR &=~1;
		i = 0;
		delay();
		LPC_GPIO0->FIOPIN &=~(1<<3);
	}
}

void EINT3_IRQHandler()
{
	if ((LPC_GPIOINT->IO0IntStatF & (1<<16)) | (LPC_GPIOINT->IO0IntStatF & (1<<17)) | (LPC_GPIOINT->IO0IntStatF & (1<<18)) | (LPC_GPIOINT->IO0IntStatF & (1<<19))){
		alim =(~LPC_GPIO0->FIOPIN & (0xF<<20));
		alim = alim >> 20;
	}
	if (alim & 0x1){// la patte 0 relie la masse, les restes sont a 1
		rep =(~LPC_GPIO0->FIOPIN & (0xF<<16));
		rep = rep >> 16;
		oldbp[0] = bp[0]; bp[0] = !(rep & 0x1); appui[0] = oldbp[0] & (!bp[0]);
		oldbp[1] = bp[1]; bp[1] = !(rep & 0x2); appui[1] = oldbp[1] & (!bp[1]);
		oldbp[2] = bp[2]; bp[2] = !(rep & 0x4); appui[2] = oldbp[2] & (!bp[2]);
		oldbp[3] = bp[3]; bp[3] = !(rep & 0x8); appui[3] = oldbp[3] & (!bp[3]);
		if (appui[0]) {commande = info[0];LPC_TIM1->TCR |= 1;}// D
		if (appui[1]) {commande = info[1];LPC_TIM1->TCR |= 1;}// C
		if (appui[2]) {commande = info[2];LPC_TIM1->TCR |= 1;}// B
		if (appui[3]) {commande = info[3];LPC_TIM1->TCR |= 1;}// A		
	}
	if (alim & 0x2){// la patte 0 relie la masse, les restes sont 1
		rep =(~LPC_GPIO0->FIOPIN & (0xF<<16));
		rep = rep >> 16;
		oldbp[4] = bp[4]; bp[4] = !(rep & 0x1); appui[4] = oldbp[4] & (!bp[4]);
		oldbp[5] = bp[5]; bp[5] = !(rep & 0x2); appui[5] = oldbp[5] & (!bp[5]);
		oldbp[6] = bp[6]; bp[6] = !(rep & 0x4); appui[6] = oldbp[6] & (!bp[6]);
		oldbp[7] = bp[7]; bp[7] = !(rep & 0x8); appui[7] = oldbp[7] & (!bp[7]);
		if (appui[4]) {commande = info[4];LPC_TIM1->TCR |= 1;}// #
		if (appui[5]) {commande = info[5];LPC_TIM1->TCR |= 1;}// 9
		if (appui[6]) {commande = info[6];LPC_TIM1->TCR |= 1;}// 6
		if (appui[7]) {commande = info[7];LPC_TIM1->TCR |= 1;}// 3
	}
	if (alim & 0x4){// la patte 0 relie la masse, les restes sont 1
		rep =(~LPC_GPIO0->FIOPIN & (0xF<<16));
		rep = rep >> 16;
		oldbp[8] = bp[8]; bp[8] = !(rep & 0x1); appui[8] = oldbp[8] & (!bp[8]);
		oldbp[9] = bp[9]; bp[9] = !(rep & 0x2); appui[9] = oldbp[9] & (!bp[9]);
		oldbp[10] = bp[10]; bp[10] = !(rep & 0x4); appui[10] = oldbp[10] & (!bp[10]);
		oldbp[11] = bp[11]; bp[11] = !(rep & 0x8); appui[11] = oldbp[11] & (!bp[11]);
		if (appui[8]) {commande = info[8];LPC_TIM1->TCR |= 1;}// 0
		if (appui[9]) {commande = info[9];LPC_TIM1->TCR |= 1;}// 8
		if (appui[10]) {commande = info[10];LPC_TIM1->TCR |= 1;}// 5
		if (appui[11]) {commande = info[11];LPC_TIM1->TCR |= 1;}// 2
	}
	if (alim & 0x8){// la patte 0 relie la masse, les restes sont 1
		rep =(~LPC_GPIO0->FIOPIN & (0xF<<16));
		rep = rep >> 16;
		oldbp[12] = bp[12]; bp[12] = !(rep & 0x1); appui[12] = oldbp[12] & (!bp[12]);
		oldbp[13] = bp[13]; bp[13] = !(rep & 0x2); appui[13] = oldbp[13] & (!bp[13]);
		oldbp[14] = bp[14]; bp[14] = !(rep & 0x4); appui[14] = oldbp[14] & (!bp[14]);
		oldbp[15] = bp[15]; bp[15] = !(rep & 0x8); appui[15] = oldbp[15] & (!bp[15]);
		if (appui[12]) {commande = info[12];LPC_TIM1->TCR |= 1;}// *
		if (appui[13]) {commande = info[13];LPC_TIM1->TCR |= 1;}// 7
		if (appui[14]) {commande = info[14];LPC_TIM1->TCR |= 1;}// 4
		if (appui[15]) {commande = info[15];LPC_TIM1->TCR |= 1;}// 1
	}
	LPC_GPIOINT->IO0IntClr = 0xF<<16;
}
int main()
{
	init_GPIO_clavier();
	init_interruption_clavier();
	init_T0_clavier();
	init_T1_clavier();
	while(1);
}