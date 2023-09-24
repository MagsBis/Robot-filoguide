#include "LPC17xx.h"


int entete = 0x2;
int numR = 0x7;
int vitesse = 0x3;    
int statut = 0x8;
int checksum = 0xE;
int message = 0x7324 ;   //0x29B6936D24DB4    0x738E
int cpt = 16;
int data = 0;
int res = 0;
int start = 0;

void init_pwm(){
LPC_PINCON -> PINSEL3 &= ~(1<<8);
LPC_PINCON -> PINSEL3 |= 1<<9;

LPC_PWM1 -> PCR &= ~(1<<2);
LPC_PWM1 -> PCR |= 1<<10;
LPC_PWM1 -> MR0 = 1000;  //TAU/5
LPC_PWM1 -> MR2 = 100;	//TAU/5/10
LPC_PWM1 -> LER |= 1<<0 | 1<<2;

LPC_PWM1 -> MCR |=  1<<1;

LPC_PWM1 -> TCR |= (1<<1);
  LPC_PWM1 -> TCR &= ~(1<<1);
LPC_PWM1 -> TCR |= (1<<0 | 1<<3);
}


void init_timer2(){
LPC_SC -> PCONP |= 1<<22;
LPC_TIM2 -> MR0 = 5000;   // tau = 200µs
LPC_TIM2 -> MR1 = 5000;
LPC_TIM2 -> MR2 = 5000;
LPC_TIM2 -> MR3 = 5000 * 150;  //tau * 50 (car il y a 50 bits)
LPC_TIM2 -> MCR |= 1<<0 | 1<<1 | 1<<3 | 1<<6 | 1<<9;
NVIC_EnableIRQ(TIMER2_IRQn);
LPC_TIM2 -> TCR |= (1<<0);
}

void  TIMER2_IRQHandler(){  


if (start == 0 ){

if (LPC_TIM2 -> IR & 1 << 2){
LPC_TIM2 -> IR |= 1<<2;
LPC_TIM2 -> MCR &= ~(1<<6);
LPC_PWM1 -> TCR &= ~(1<<0 | 1<<3);
LPC_PINCON -> PINSEL3 &= ~(3<<8);
LPC_GPIO1 -> FIODIR |= 1<<20;
LPC_GPIO1 -> FIOPIN &= ~(1<<20);

start = start +1;
}
}

if (start >= 1){

if (res < 3){

if (LPC_TIM2 -> IR & 1 << 0){
LPC_TIM2 -> IR |= 1<<0;
LPC_TIM2 -> MR0 = 15000;  //3TAU
start =  start + 1;
cpt = cpt - 1;
data = message & (1<<cpt);
data = data >> cpt;

if (data){
LPC_TIM2 -> MR1 = 10000;  //2TAU
LPC_PINCON -> PINSEL3 &= ~(1<<8);
LPC_PINCON -> PINSEL3 |= 1<<9;
LPC_PWM1 -> TCR |= (1<<0 | 1<<3);
}
if (!(data)){
LPC_TIM2 -> MR1 = 5000;  //TAU
LPC_PINCON -> PINSEL3 &= ~(1<<8);
LPC_PINCON -> PINSEL3 |= 1<<9;
LPC_PWM1 -> TCR |= (1<<0 | 1<<3);
}
if (cpt == -1){
start = 0;
LPC_PINCON -> PINSEL3 &= ~(1<<8);
LPC_PINCON -> PINSEL3 |= 1<<9;
LPC_PWM1 -> TCR |= (1<<0 | 1<<3);
LPC_TIM2 -> TCR |= 1<<1;
LPC_TIM2 -> TCR &= ~(1<<1);
LPC_TIM2 -> MCR |= (1<<6);
LPC_TIM2 -> MR0 = 5000;  //TAU
LPC_TIM2 -> MR1 = 0;  //0
cpt = 16;
res = res +1;
}

}
if (LPC_TIM2 -> IR & 1<<1){
LPC_TIM2 -> IR |= 1<<1;
LPC_PWM1 -> TCR &= ~(1<<0 | 1<<3);
LPC_PINCON -> PINSEL3 &= ~(3<<8);
LPC_GPIO1 -> FIODIR |= 1<<20;
LPC_GPIO1 -> FIOPIN &= ~(1<<20);

}

if (res >= 3) {  
LPC_PWM1 -> TCR &= ~(1<<0 | 1<<3);
LPC_TIM2 -> TCR |= 1<<1;
LPC_TIM2 -> TCR &= ~(1<<1);
LPC_TIM2 -> MCR &= ~(1<<1);
LPC_PINCON -> PINSEL3 &= ~(3<<8);
LPC_GPIO1 -> FIODIR |= 1<<20;
LPC_GPIO1 -> FIOPIN &= ~(1<<20);
start = -1;
}
}
}
if (LPC_TIM2 -> IR & 1 << 3 ){
LPC_TIM2 -> IR |= 1<<3;
start = 0;
res = 0;
LPC_TIM2 -> TCR |= 1<<1;
LPC_TIM2 -> TCR &= ~(1<<1);
LPC_TIM2 -> MCR |= 1<<1;
LPC_TIM2 -> MCR |= (1<<6);
LPC_TIM2 -> MR2 = 5000;   //TAU
LPC_PINCON -> PINSEL3 &= ~(1<<8);
LPC_PINCON -> PINSEL3 |= 1<<9;
LPC_PWM1 -> TCR |= (1<<0 | 1<<3);
}
}


int main(){

init_pwm();
init_timer2();

while(1){
}
return 0;
}
