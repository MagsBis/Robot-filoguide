#include "LPC17xx.h"
int t_original_dtmf[101] = {512,544,576,608,639,670,701,730,759,786,813,838,863,885,907,926,944,961,975,
							988,999,1008,1015,1020,1023,1024,1023,1020,1015,1008,999,988,975,961,944,926,907,
							885,863,838,813,786,759,730,701,670,639,608,576,544,512,480,448,416,385,354,324,
							294,265,238,211,186,162,139,118,98,80,63,49,36,25,16,9,4,1,0,1,4,9,16,25,36,
							49,63,80,98,118,139,162,186,211,238,265,294,324,354,385,416,448,480,512};
/*
int t_original_dtmf[37] = {511,611,707,795,872,936,983,1012,1022,1012,983,936,872,795,707,611,
	511,411,315,227,150,86,39,10,0,10,39,86,150,227,315,411,511,611,707,795,511};
*/
							
int Low_freq_num[10] = {941,697,697,697,770,770,770,852,852,852};
int High_freq_num[10] = {1336,1209,1336,1477,1209,1336,1447,1209,1336,1447}; 
int seq_freq_haute[12],seq_freq_basse[12];
int val_son_1,val_son_2,val_son;
int ech_avant_H,ech_apres_H,ech_avant_L,ech_apres_L;
int numero = 01,indice = 1,decompteur = 10000;
int index_son;
int index_a_virgule_H,index_a_virgule_L;
int index_H,index_L;
int virgule_H,virgule_L;
int freq_haute,freq_basse;
int dizaine_num_low,dizaine_num_high,unite_num_low,unite_num_high;
int DTMF_ON;
	
void init_T1_dtmf()
{
	LPC_TIM1->MR0 = 25000000/100000;
	LPC_TIM1->MCR = 3;
	NVIC_EnableIRQ(TIMER1_IRQn);
	LPC_TIM1->TCR =1;
}

void init_dtmf()
{
	dizaine_num_low = Low_freq_num[numero/10];
	dizaine_num_high = High_freq_num[numero/10];
	unite_num_low = Low_freq_num[numero%10];
	unite_num_high = High_freq_num[numero%10];
	
	seq_freq_basse[0] = 0.941*256;
	seq_freq_basse[1] = 0.941*256;
	seq_freq_basse[2] = 0;
	seq_freq_basse[3] = dizaine_num_low*0.256;
	seq_freq_basse[4] = dizaine_num_low*0.256;
	seq_freq_basse[5] = 0;
	seq_freq_basse[6] = unite_num_low*0.256;
	seq_freq_basse[7] = unite_num_low*0.256;
	seq_freq_basse[8] = 0;
	seq_freq_basse[9] = 0.941*256;
	seq_freq_basse[10] = 0.941*256;
	seq_freq_basse[11] = 0;
	
	seq_freq_haute[0] = 1.209*256; 
	seq_freq_haute[1] = 1.209*256;
	seq_freq_haute[2] = 0;
	seq_freq_haute[3] = dizaine_num_high*0.256;
	seq_freq_haute[4] = dizaine_num_high*0.256;
	seq_freq_haute[5] = 0;
	seq_freq_haute[6] = unite_num_high*0.256;
	seq_freq_haute[7] = unite_num_high*0.256;
	seq_freq_haute[8] = 0;
	seq_freq_haute[9] = 1.447*256;
	seq_freq_haute[10] = 1.447*256;
	seq_freq_haute[11] = 0;
}

void init_DAC(){
	LPC_PINCON->PINSEL1 &= ~(1<<20);
	LPC_PINCON->PINSEL1 |= 1<<21;
}
void TIMER1_IRQHandler()
{
	LPC_TIM1->IR	|= 1;
	decompteur--;
	
	if(decompteur == 0){
		decompteur = 10000;
		index_son++;
	}
	if(index_son == 12){index_son = 0;}
	freq_haute = seq_freq_haute[index_son];
	freq_basse = seq_freq_basse[index_son];

	index_a_virgule_H =  index_a_virgule_H + freq_haute;
	index_a_virgule_L =  index_a_virgule_L + freq_basse;
		
	if(index_a_virgule_H >= 25600)index_a_virgule_H = index_a_virgule_H - 25600;
	if (index_a_virgule_L >= 25600)index_a_virgule_L = index_a_virgule_L - 25600;
		
	index_H = index_a_virgule_H >>8;
	index_L = index_a_virgule_L >>8;
		
	virgule_H = index_a_virgule_H  & 0XFF;
	virgule_L = index_a_virgule_L  & 0XFF;
		
	ech_avant_H = t_original_dtmf[index_H];
	ech_apres_H = t_original_dtmf[index_H + 1];
		
	ech_avant_L = t_original_dtmf[index_L];
	ech_apres_L = t_original_dtmf[index_L + 1];
	
	val_son_1 = (ech_avant_H*(256 - virgule_H) + ech_apres_H*virgule_H)>>8;
	val_son_2 = (ech_avant_L*(256 - virgule_L) + ech_apres_L*virgule_L)>>8;
	
	val_son = (val_son_1 + val_son_2)>>1;
	
	LPC_DAC->DACR = val_son<<6;
}
int main()
{
	init_dtmf();
	init_T1_dtmf();
	init_DAC();
	while(1){}
}