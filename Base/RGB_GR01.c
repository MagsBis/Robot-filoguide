/////////LED RGB QUI AFFICHE LE STATUT DU ROBOT\\\\\\\\\

int statut = 0x1;

//P0.8 = rouge, P0.9 = bleu ,  P0.10 = vert  

void init_gpio(){
	LPC_PINCON -> PINSEL0 &= ~(3<<16 | 3<<18 | 3<<20);
	LPC_GPIO0 -> FIODIR |= (1<<8 | 1<<9 | 1<<10);

	if (statut == 0x8){
			LPC_GPIO0 -> FIOPIN |= 1<<10;
			LPC_GPIO0 -> FIOPIN &= ~(1<<8 | 1<<9);
	}
	
	if (statut == 0x4){
			LPC_GPIO0 -> FIOPIN |= 1<<8;
			LPC_GPIO0 -> FIOPIN &= ~(1<<10| 1<<9);
	}
	
	if (statut == 0x2){
			LPC_GPIO0 -> FIOPIN |= 1<<8 | 1<< 9;
			LPC_GPIO0 -> FIOPIN &= ~(1<<10);
	}
	
	if (statut == 0x1){
			LPC_GPIO0 -> FIOPIN |= 1<<9;
			LPC_GPIO0 -> FIOPIN &= ~(1<<8 | 1<<10);
	}
}	
