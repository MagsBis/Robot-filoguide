```
// Par : Chems

/************ Périphériques utilisés (pour le moment): ************************
 * EINT3 (GPIO interrupt) : P0.2 (BOBV_NUM), P1.28-31 (NUM_ROBOT) 
 * TIMER 0
 *********************************************************************/

/************** Fonctions et structs implémentées : *************************
 * struct COMMANDE : information concernants la commande reçue
 * DECODE_FIL() : décoder un message reçu par le fil
 * Machine à état du robot (pas encore fini)
 */

/*********** À faire : ****************
 * Creer une struct ROBOT qui contiendra le numéro, le statut, la vitesse et d'autres caractèristiques
 * Finir la MAE
 * voir autres trucs
 */
 
#include "LPC17xx.h"

void UPDATE_VIT();
int TOP_2 = 0; // Pour stocker la valeur du timer à chaque front descendant de BOBV_NUM    -Chems
int bit_counter = 13; // Compteur de bit que je décrémenterai à chaque lecture de bit sur BOBV_NUM  -Chems
int MESSAGE_READY = 0; // Flag levé quand un message reçu est valide  -Chems
unsigned int MESSAGE = 0; // Le message reçu de la base -Chems
int DTMF_Flag = 0; // Flag incrémenté à chaque symbole reçu en DTMF
/* Struct contenant les informations du robot, mis à jour par les messages envoyés de la base */
typedef struct {
	int MISSION;
	int NUM;
	int VITG;
	int VITD;
	int VITMOY;
	int LETTRE;
	int DEST;
	int POSTE;
	int STATUS;
}ROBOT;
ROBOT R = {0, 0, 30, 30, 40, 0, 0, 0, 0}; // Initialisation à 0;
int ETAT = 0; // Initialisation de l'état à 0 (IDLE)

/* Différents états possibles :
	- 0 : IDLE : état initiale de la MAE : 
	Le robot roule sur le circuit à une vitesse, un statut, une destination, une mission, différentes transitions possible
		+ Si le numéro du robot reçu dans MESSAGE == NUM_ROBOT, passage à CHANGE_DIRECTIVE
		+ Si Destination atteinte, se garer à droite ou à gauche en fonction du dernier MESSAGE
		+ Si déviation (pas encore implémenté), asservir les vitesses des 2 moteurs gauche et droite
        - 1 : CHANGE_DIRECTIVE : Décoder le MESSAGE reçu (voir DECODE_FIL()) et changer les propriétés du robot, revenir à IDLE après
	- 2 : GARER : Se garer à droite ou à gauche en fonction de mission, beep en fonction de mission, attendre un appui du bouton
	- 3 : DEMARRER : Démarrer vers la droite ou la gauche en fonction de mission, revenir à IDLE juste après
	- 4 : Asservir : pas encore implémenté. But : remettre le robot sur le fil
*/


void init_PIN_ROBOT() {
	// Configuration de TOUS les pins qu'on a utilisé dans l'excel
	// Merci le debugger
	LPC_PINCON->PINSEL1 = 0x00054000; 
	LPC_PINCON->PINSEL4 = 0x01100045;
	
	LPC_PINCON->PINMODE0 = 0xC0FFFFF3 ;
	LPC_PINCON->PINMODE1 = 0x3FFFFFFF;
	LPC_PINCON->PINMODE2 = 0xFFFFFFFF;
	LPC_PINCON->PINMODE3 = 0x0CCFFF30;
	LPC_PINCON->PINMODE4 = 0x03300000;
	LPC_PINCON->PINMODE7 |= 0x000C0000;
	LPC_PINCON->PINMODE9 = 0x0F000000;
	
	LPC_GPIO0->FIODIR = 0x08400FF1;
	LPC_GPIO1->FIODIR = 0x001FC411;
	LPC_GPIO2->FIODIR = 0x000003CB;
	LPC_GPIO3->FIODIR |= 0x06000000;
	
	LPC_SC->PCONP |= 3 << 22;
	LPC_SC->PCONP |= 3 << 12;
	
}


/* Interrupt Handler destiné à décoder le message reçu sur le fil */
void EINT3_IRQHandler() {
	if(LPC_GPIOINT->IO0IntStatR & 2) { 
		// si interruption sur front montant du P0.2, lancer le timer et sortir de la fonction
		LPC_GPIOINT->IO0IntClr |= 2;
		LPC_TIM0->TCR = 1;
		return;
	}
	else if (LPC_GPIOINT->IO0IntStatF & 2) {
		// sinon stopper le timer et le sauvegarder dans TOP_2
		LPC_GPIOINT->IO0IntClr |= 2;
		TOP_2 = LPC_TIM0->TC;
		LPC_TIM0->TCR = 2;
		// Décoder si c'est une entête ou un 1 ou un 0 en fonction de la valeur de TOP_2
		// Je me suis permis de laisser une marge d'erreur de 300µs
		if((TOP_2 >= 25E6*22E-4) && (TOP_2 <= 25E6*28E-4)) {
			// Mettre à l'entête et reset le bit counter
			MESSAGE |= 1 << 15;
			MESSAGE &= ~(1 << 14);
			bit_counter = 13;
		}
		if((TOP_2 >= 25E6*7E-4) && (TOP_2 <= 25E6*13E-4)) {
			//Mettre un 0 et décrémenter le bit_counter, comme le message n'est pas fini, on s'assure que MESSAGE_READY = 0
			MESSAGE &= ~(1 << bit_counter);
			bit_counter--;
			MESSAGE_READY = 0;
		}
		if((TOP_2 >= 25E6*14E-4) && (TOP_2 <= 25E6*2E-3)) {
			// Même chose qu'en haut met pour un 1
			MESSAGE |= (1 << bit_counter);
			bit_counter--;
			MESSAGE_READY = 0;
		}
		if(bit_counter == -1) { 
			// Si on a fini un message de 16 bits, on reset bit_counter et on met MESSAGE_READY à 1
			bit_counter = 13;
			MESSAGE_READY = 1;
			// Si le message contient le même numéro du robot que le notre, changer d'état vers CHANGE_DIRECTIVE
			if (((MESSAGE & (0xF << 7)) >> 7) == R.NUM) {
				ETAT = 1;
			}
		}
	}
	// Pour le moment, je m'en fous de le l'angle et la distance du fil, j'essaie d'asservir à
	// Interruption sur Front montant du COTE_BOBH1
	else if ((LPC_GPIOINT->IO0IntStatR & (1<<9))) {
		LPC_GPIOINT->IO0IntClr |= 1<<9;
		// Voir si l'amplitude du signal est non négligeable, valeur à déterminer après
		//if(LPC_ADC->ADDR1 > 2500) {
		// Voir le coté de l'autre bobine (on sait que la bobine 1 est en inversion de phase par rapport à BOBV)
		if(LPC_GPIO0->FIOPIN & (1 << 8)) { // bobine H2 en inversion de phase aussi
			// Moteur droite doit rouler beaucoup plus vite que moteur gauche
			// Pour le moment, je pose VITG = 30 et VITD = 70
			R.VITD = 70*R.VITMOY/80;
			R.VITG = 30*R.VITMOY/80;
			UPDATE_VIT();
			return; // Sortir du Handler
		}
		// Sinon, bobine 2 en phase
		// Moteur droite doit rouler plus vite que moteur gauche
			R.VITD = 60*R.VITMOY/80;
			R.VITG = 40*R.VITMOY/80;
			UPDATE_VIT();
		//}
	}
	// Interruption sur front descendant du COTE_BOBH1
	else if ((LPC_GPIOINT->IO0IntStatF & (1<<9))) {
		LPC_GPIOINT->IO0IntClr |= 1<<9;
		
		// Voir si l'amplitude du signal est non négligeable, valeur à déterminer après
		//if(LPC_ADC->ADDR1 > 2500) {
		// Voir le coté de l'autre bobine (on sait que la bobine 1 est en phase par rapport à BOBV)
		if(LPC_GPIO0->FIOPIN & (1 << 8)) { // bobine H2 en inversion de phase 
			// Moteur droite doit roule moins vite que moteur gauche
			// Pour le moment, je pose VITG = 30 et VITD = 70
			
			R.VITD = 60*R.VITMOY/80;
			R.VITG = 40*R.VITMOY/80;
			UPDATE_VIT();
			return; // Sortir du Handler
		}
		// Sinon, bobine 2 en phase
		// Moteur droite doit rouler beaucoup moins vite que moteur gauche
			R.VITD = 70*R.VITMOY/80;
			R.VITG = 30*R.VITMOY/80;
			UPDATE_VIT();
		//}
	}
	// Interruption sur front montant du COTE_BOBH2
	else if ((LPC_GPIOINT->IO0IntStatR & (1 << 8))) {
		LPC_GPIOINT->IO0IntClr |= 1 << 8;
		// Voir le coté de l'autre bobine (on sait que la bobine 2 est en inversion de phase par rapport à BOBV)
		//if(LPC_ADC->ADDR2 > 2500) {
		if(LPC_GPIO0->FIOPIN & 1 ) { // bobine H1 en inversion de phase 
			// Moteur droite doit roule moins vite que moteur gauche
			// Pour le moment, je pose VITG = 30 et VITD = 70
			
			R.VITD = 70*R.VITMOY/80;
			R.VITG = 30*R.VITMOY/80;
			UPDATE_VIT();
			return; // Sortir du Handler
		}
		// Sinon, bobine H1 en phase
		// Moteur droite doit rouler moins vite que moteur gauche
			R.VITD = 40*R.VITMOY/80;
			R.VITG = 60*R.VITMOY/80;
			UPDATE_VIT();
		//}
	}
	// Interruption sur front descendant du COTE_BOBH2
	else if ((LPC_GPIOINT->IO0IntStatF & (1 << 8))) {
		LPC_GPIOINT->IO0IntClr |= 1 << 8;
		// Voir le coté de l'autre bobine (on sait que la bobine 2 est en phase par rapport à BOBV)
		//if(LPC_ADC->ADDR2 > 2500) {
		if(LPC_GPIO0->FIOPIN & (1<<9) ) { // bobine H1 en inversion de phase 
			// Moteur droite doit roule plus vite que moteur gauche
			// Pour le moment, je pose VITG = 30 et VITD = 70
			
			R.VITD = 60*R.VITMOY/80;
			R.VITG = 40*R.VITMOY/80;
			UPDATE_VIT();
			return; // Sortir du Handler
		}
		// Sinon, bobine H1 en phase
		// Moteur droite doit rouler beaucoup moins vite que moteur gauche
			R.VITD = 40*R.VITMOY/80;
			R.VITG = 60*R.VITMOY/80;
		UPDATE_VIT();
		//}
	}
}

/* Initialisation de l'interruption sur DV (DTMF signal reçu valide) */
void init_EINT2() {
	LPC_SC->EXTMODE |= 1 << 2;
	LPC_SC->EXTPOLAR |= 1 << 2;
	LPC_SC->EXTINT |= 1 << 2;
	NVIC_EnableIRQ(EINT2_IRQn);
}

void EINT2_IRQHandler() {
	
	LPC_SC->EXTINT |= 1 << 2; // Acquitter l'interruption
//	if(((LPC_GPIO0->FIOPIN & (15<<16))>>16) == 1) {
//		LPC_PWM1->MR1 = 0;
//		LPC_PWM1->MR2 = 0;
//		LPC_PWM1->LER |= 6;
//	}
//	if(((LPC_GPIO0->FIOPIN & (15<<16))>>16) == 2) {
//		LPC_PWM1->MR1 = 12500*R.VITG/100;
//	LPC_PWM1->MR2 = 12500*R.VITD/100;
//	LPC_PWM1->LER |= 6;
//	}
	if ( DTMF_Flag == 0) {
		// Le premier symbole reçu doit être une étoile
		// Correspond à la valeur 11
		if (((LPC_GPIO0->FIOPIN & (15<<16))>>16)==11){
			DTMF_Flag++;
		}
	}
	else if (DTMF_Flag == 1) {
		// Le deuxième symbole reçu doit être le numéro des dizaines
		R.POSTE = ((LPC_GPIO0->FIOPIN & (15<<16))>>16)*10;
		DTMF_Flag++;
	}
	else if (DTMF_Flag = 2) {
		R.POSTE = R.POSTE + ((LPC_GPIO0->FIOPIN & (15<<16))>>16);
		DTMF_Flag++;
	}
	else if (DTMF_Flag = 3) {
		if (((LPC_GPIO0->FIOPIN & (15<<16))>>16)==12)
			{
				LPC_GPIO1->FIOPIN = R.POSTE<<15; //décalge a gauche pour faire correspondre au 5 leds mis à disposition
				DTMF_Flag=0;   	 //réinitialisation pour pouvoir acquérir le numéro d'autre poste
			}
	}
	if(R.POSTE == R.DEST) {
		ETAT = 2;
	}
	
}
/* Fonction d'initialisation de l'interruption sur BOBV_NUM (P0.2) */
void init_INT_BOBV_NUM() {
	LPC_GPIOINT->IO0IntEnR |= 2;
	LPC_GPIOINT->IO0IntEnF |= 2;
	LPC_GPIOINT->IO0IntClr |= 2;
	NVIC_EnableIRQ(EINT3_IRQn);
}

void init_INT_COTE_BOBH() {
	LPC_GPIOINT->IO0IntEnR |= 1 << 8;
	LPC_GPIOINT->IO0IntEnF |= 1 << 8;
	
	LPC_GPIOINT->IO0IntEnR |= 1 << 9;
	LPC_GPIOINT->IO0IntEnF |= 1 << 9;
}

void UPDATE_VIT() {
	// Régler les PWM gauche et droite pour mettre à jour la vitesse après une commande de vitesse reçu
	LPC_PWM1->MR1 = 12500*R.VITG/100;
	LPC_PWM1->MR2 = 12500*R.VITD/100;
	LPC_PWM1->LER |= 6;
	return;
}

void UPDATE_STATUS() {
	// En fonction R.MISSION et R.STATUS, Mettre à jour R.STATUS
	return;
}

void init_MOT() {
	LPC_PWM1->PCR |= 3 << 9;
	LPC_PWM1->MR0 = 12500;
	LPC_PWM1->MR1 = R.VITG*12500/100;
	LPC_PWM1->MR2 = R.VITD*12500/100;
	LPC_PWM1->MCR |= 2;
	LPC_PWM1->TCR |= 1 | (1<<3);
	
}
/* Fonction du décodage du message reçu, à executer en état CHANGE_DIRECTIVE */
void DECODE_FIL() {
	if (MESSAGE_READY == 0) { // S'assurer que le message est prêt, quasi toujours le cas
		return;
	}
	/* Switch sur le type de message
	    - 0 : commande de vitesse
	    - 1,2,3,4 : commande de chargement/déchargement voir cahier de charges
	*/
	
	switch ((MESSAGE & (7 << 11)) >> 11) { 
		case 0:
			R.NUM = (MESSAGE & (0xF << 7)) >> 7; 
			R.MISSION = 0; 
			R.VITMOY = MESSAGE & 0x3F;
			UPDATE_VIT();
			break;
		case 1: 
			R.NUM = (MESSAGE & (0xF << 7)) >> 7; 
			R.MISSION = 1; 
			R.LETTRE = (MESSAGE & (3 << 5)) >> 5; 
			R.DEST = MESSAGE & 0x1F;
			UPDATE_STATUS();
			break;
		case 2: 
			R.NUM = (MESSAGE & (0xF << 7)) >> 7; 
			R.MISSION = 2; 
			R.LETTRE = (MESSAGE & (3 << 5)) >> 5; 
			R.DEST = MESSAGE & 0x1F;
			UPDATE_STATUS();
			break;
		case 3: 
			R.NUM = (MESSAGE & (0xF << 7)) >> 7; 
			R.MISSION = 3; 
			R.LETTRE = (MESSAGE & (3 << 5)) >> 5; 
			R.DEST = MESSAGE & 0x1F;
			UPDATE_STATUS();
			break;
		case 4: 
			R.NUM = (MESSAGE & (0xF << 7)) >> 7; 
			R.MISSION = 4; 
			R.LETTRE = (MESSAGE & (3 << 5)) >> 5; 
			R.DEST = MESSAGE & 0x1F;
			UPDATE_STATUS();
			break;
		// mode debug à implémenter après
		default : return;	
	}
	MESSAGE_READY = 0; // Message décodé, remêtre message prêt à 0
	ETAT = 0; // Revenir à IDLE
}
// Lire le numéro du robot
void GET_NUM_ROBOT() {
	R.NUM = (LPC_GPIO1->FIOPIN & 0xF << 28) >> 28;
}

/* Initialiser le composant ADC */
void init_ADC() {
	LPC_ADC->ADCR |= 3; // Utiliser AD0.0 -> AD0.2
	LPC_ADC->ADCR |= 0xFF << 8; // Diviser l'horloge par 2^8 (~100KHz), osef parceque on va avoir un détécteur de crête qui va nous donner l'amplitude
	LPC_ADC->ADCR |= 1 << 16; // Mettre BURST à 1
	LPC_ADC->ADCR |= 1 << 21; // Mettre le convertisseur en marche
}

void init_ARRET_URGENCE() {
		// Initialisation de l'interruption de P2.10
	LPC_SC->EXTMODE |= 1; 				// Interruption sur front
	LPC_SC->EXTPOLAR &= 0; 				// Descendant
	LPC_SC->EXTINT |= 1; 					// Acquittement de l'interruption
	NVIC_EnableIRQ(EINT0_IRQn);	//Autorisation de l'interruption
}

void EINT0_IRQHandler(){
	
	LPC_SC->EXTINT |= 1; 			// Acquittement de l'interruption
	// Remplacer par autre chose de plus efficace
	LPC_PINCON->PINSEL0 = 0;	// GPIO sur tous les ports
	LPC_PINCON->PINSEL1 = 0;
	LPC_PINCON->PINSEL2 = 0;
	LPC_PINCON->PINSEL3 = 0;
	LPC_PINCON->PINSEL4 = 0;
	LPC_PINCON->PINSEL5 = 0;
	LPC_PINCON->PINSEL6 = 0;
	LPC_PINCON->PINSEL7 = 0;
	LPC_PINCON->PINSEL8 = 0;
	LPC_PINCON->PINSEL9 = 0;
	
	LPC_GPIO0->FIOPIN = 1;		// Mise a 0 de toutes les pates
	LPC_GPIO1->FIOCLR = 1;		
	LPC_GPIO2->FIOCLR = 1;
	LPC_GPIO3->FIOCLR = 1;
	LPC_GPIO4->FIOCLR = 1;
	while(1){
	}
}

void GARER() {
	if(R.MISSION == 1) {
		
	}
	else if (R.MISSION == 2) {
		
	}
	else if (R.MISSION == 3) {
		
	}
	else if (R.MISSION == 4) {
		
	}
}
int main() {
	
	init_PIN_ROBOT();
	init_INT_BOBV_NUM();
	init_INT_COTE_BOBH();
	GET_NUM_ROBOT();
	init_ADC();
	init_MOT();
	init_EINT2();
	//4init_ARRET_URGENCE();
	while(1) {
		switch(ETAT) {
			case 0 : 
				break;
			case 1 : 
				DECODE_FIL(); 
				break;
			case 2 : 
				GARER();
				break;
			case 3 : break;
			case 4 : break;
		}
	}
	
	return 0;
	
}

```