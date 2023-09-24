 #include "LPC17xx.h"

int occ=0;

//VALEURS SIMU ET UTILISER ECHELLE 0.1s dans logic analyzer
//int tau=25E5;//25E6*0.1
//int tauMin=15E5;//60E-3*25E6;
//int tauMax=856E4;//140E-3*25E6;
//int DeuxTauMin=40E5;//160E-3*25E6;
//int DeuxTauMax=60E5;//240E-3*25E6;
//int TroisTauMin=65E5;//260E-3*25E6;
//int TroisTauMax=85E5;//340E-3*25E6;

//VALEURS EXPERIMENTALES
int tau = 25*200;//float tau=200E-6;
int tauMax = 25*280;//float tauMin=150E-6;
int tauMin = 25*120;//float tauMax=250E-6;
int DeuxTauMin = 25*320;//float DeuxTauMin=350E-6;
int DeuxTauMax = 25*480;//float DeuxTauMax=450E-6;
int TroisTauMin = 25*520;//float TroisTauMin=550E-6;
int TroisTauMax = 25*680;//float TroisTauMax=650E-6;

int MemoErreur=1;
int CapR;
int CapF;
int indice=15;
int data;
int flagTrame=0;
int flagEntete=0;
int checksum;
int num;
int vitesse;
int statut;
int checksum_eff;
int trame;

void PWM1_IRQHandler(){
	LPC_PWM1->IR |= 1;
	
	LPC_GPIO2->FIOCLR = 0xFFF;
}

void TIMER1_IRQHandler(){
	LPC_TIM1->IR |= 1;
	
	//BAISSE PATTE TRAME VALIDE
		LPC_GPIO2->FIOPIN &= ~(1<<13);
		flagTrame=0;
}

void verif_checksum(){
	//EXTRACTION DES INFOS INDIVIDUELLES
	checksum = trame;
	
	num = trame;
	
	vitesse = trame;
	
	statut = trame;
	
	num = num & (0xF<<12);
	num = num>>12;
	
	vitesse = vitesse & (0xF<<8);
	vitesse = vitesse>>8;
	
	statut = statut & (0xF<<4);
	statut = statut>>4;
	
	checksum = checksum & (0xF);
	
	//VERIF CHECKSUM
	checksum_eff=num+vitesse+statut+checksum;
	checksum_eff= checksum_eff & (0xF);
	
	//CHECKSUM CORRECTE
	if(checksum_eff==0){
		
		//LEVE PATTE TRAME VALIDE
		flagTrame = 1;
		LPC_GPIO2->FIOPIN |= 1<<13;

		//START TIMER 2s pour trame valide
		LPC_TIM1->TCR |= 1;
		
		//START AFFICHAGE 10s infos robot : num vitesse statut
		trame = trame >> 4;
		LPC_GPIO2->FIOCLR = 0xFFF;
		LPC_GPIO2->FIOPIN |= trame;
		LPC_PWM1->TCR |= 1;
		
			//LEVE PATTE ERREUR
		MemoErreur=1;
		LPC_GPIO0->FIOPIN |= 1<<11;
		
		}else{	//CHECKSUM INCORRECTE
		//LEVE PATTE ERREUR
		MemoErreur=1;
		LPC_GPIO0->FIOPIN |= 1<<11;
		
		//INIT INDICE
		indice=15;
		LPC_GPIO2->FIOCLR = 0xFFF;
		
		//RESET TRAME
		trame=0;
	}
}

void verif_entete_motif(){
	//IDENTIFICATION Entête
	if ( ( (CapF <= tauMax) && (CapF >=tauMin)) && (CapR <= DeuxTauMax) && (CapR >= DeuxTauMin)) {
		indice=15;
		
		//LEVE PATTE ENTETE
		flagEntete = 1;
		LPC_GPIO2->FIOPIN |= 1<<12;
		
		//BAISSE PATTE ERREUR
		MemoErreur=0;
		LPC_GPIO0->FIOPIN &= ~(1<<11);
		
		//SIMU : CLEAR MESSAGE
		LPC_GPIO1->FIOCLR = 0xFFFF<<16;
	}else{
			//IDENTIFICATION data=1
			if ( (MemoErreur == 0) && ((CapF <= DeuxTauMax) && (CapF >= DeuxTauMin)) && ((CapR <= TroisTauMax) && (CapR >= TroisTauMin))){	
				data = 1;
				
				//SIMU : VISUALISATION DIRECTE MESSAGE SUR P.16 A P.31
				//LPC_GPIO1->FIOPIN |= data<<(indice+16);
				
				//MAJ TRAME
				trame |= data<<indice;
				indice--;
			}else{
				//IDENTIFICATION data=0
				if ( (MemoErreur == 0) && ((CapF <= tauMax) && (CapF >= tauMin)) && ((CapR <= TroisTauMax) && (CapR >= TroisTauMin))){	
				data = 0;
					
					//SIMU : VISUALISATION DIRECTE MESSAGE SUR P.16 A P.31
					//LPC_GPIO1->FIOPIN &= ~(1<<(indice+16));
					
					//MAJ TRAME
					trame &= ~(1<<indice);
					indice--;
				}else{
					//LEVE PATTE ERREUR
					MemoErreur=1;
					LPC_GPIO0->FIOPIN |= 1<<11;
					
					//BAISSE PATTE ENTETE
					flagEntete=0;
					LPC_GPIO2->FIOPIN &= ~(1<<12);
					
					//SIMU : CLEAR MESSAGE
					LPC_GPIO1->FIOCLR = 0xFFFF<<16;
					
					//init indice
					indice=15;
					trame = 0;
				}
			}
			if(indice==-1){
				//BAISSE PATTE ENTETE
				flagEntete=0;
				LPC_GPIO2->FIOPIN &= ~(1<<12);
				
				indice=15;
	
				verif_checksum();

			}
		}
}


void EINT3_IRQHandler(){
	
	//MESURE DUREE ENTRE RISING EDGE ET RISING EDGE
	if(LPC_GPIOINT->IO0IntStatR & (1<<24)){
		//if(occ==1){
			CapR = LPC_TIM0->TC;
			verif_entete_motif();
		//}
			//copie trame (1)
			LPC_GPIO0->FIOPIN |= 1<<10;
		
			//Reinit tim0
			LPC_TIM0->TCR |= 2;
			
			//Lancer tim0
			LPC_TIM0->TCR &= ~2;
			LPC_TIM0->TCR |= 1;	
			LPC_GPIOINT->IO0IntClr |= 1<<24;
	}
	
	//MESURE DUREE ENTRE RISING EDGE ET FALLING EDGE
	if (LPC_GPIOINT->IO0IntStatF & (1<<24)){
		//copie trame (0)
		LPC_GPIO0->FIOPIN &= ~(1<<10);
		
		CapF = LPC_TIM0->TC;
		occ=1;
		LPC_GPIOINT->IO0IntClr |= 1<<24;
	}
}


void init_GPIO(){
	//P0.10 EN SORTIE copie trame
	LPC_PINCON->PINSEL0 &= ~(3<<20);
	LPC_GPIO0->FIODIR |= 1<<10;
	LPC_GPIO0->FIOPIN &= ~(1<<10);
	
	//P0.11 EN SORTIE (Erreur)
	LPC_PINCON->PINSEL0 &= ~(3<<22);
	LPC_GPIO0->FIODIR |= 1<<11;
	LPC_GPIO0->FIOPIN |= 1 << 11; //erreur à l'initialisation
	
	//P0.24 EN ENTREE (Acquisition trame)
	LPC_PINCON->PINSEL1 &= ~(3<<16);
	LPC_GPIO0->FIODIR &= ~(1<<24);
	
	//P1.16->P1.31 EN GPIO
	LPC_PINCON->PINSEL3 &= ~(0xFFFFFF);
	LPC_GPIO1->FIODIR |= 0xFFFF<<16;;
	LPC_GPIO1->FIOCLR = 0xFFFF<<16;
	
	//P2.13 EN SORTIE (FLAG TRAME)
	LPC_PINCON->PINSEL4 &= ~(3<<26);
	LPC_GPIO2->FIODIR |= 1<<13;
	LPC_GPIO2->FIOCLR = 1<<13;
	
	//P2.12 EN SORTIE (ENTETE)
	LPC_PINCON->PINSEL4 &= ~(3<<24);
	LPC_GPIO2->FIODIR |= 1<<12;
	LPC_GPIO2->FIOCLR = 1<<12;
	
	//P2.0 A P2.11 EN SORTIE (NUM VITESSE STATUT)
	LPC_PINCON->PINSEL4 &=~(0xFFFFFF);
	LPC_GPIO2->FIODIR |= 0xFFF;
	LPC_GPIO2->FIOPIN &= ~(0xFFF);
}

void init_t0(){
	LPC_TIM0->MR0 = 25E6*4*tau;
	LPC_TIM0->MCR |= 2;
}

void init_t1(){
	LPC_TIM1->MR0 = 25E6*2;
	LPC_TIM1->MCR |= 7;
	NVIC_EnableIRQ(TIMER1_IRQn);
}

void init_pwm(){
	LPC_PINCON->PINSEL3 &= ~(1<<4);
	LPC_PINCON->PINSEL3 |= 1<<5;
	
	LPC_PWM1->PCR |= 1<<9;
	
	LPC_PWM1->MR0 = 25E6*10;
	
	LPC_PWM1->LER |= 1;
	
	LPC_PWM1->MCR |= 7;
	
	NVIC_EnableIRQ(PWM1_IRQn);
}

void init_IR(){
	LPC_GPIOINT->IO0IntEnR |= 1<<24;
	LPC_GPIOINT->IO0IntEnF |= 1<<24;
	NVIC_EnableIRQ(EINT3_IRQn);
}

int main(){
	init_GPIO();
	init_t0();
	init_t1();
	init_pwm();
	init_IR();
	
	while(1);
	
	return 0;
}

