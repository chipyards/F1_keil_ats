#include "audio.h"
#include "gpio.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"

type_etat etat;

// callback pour l'interrupt audio, duree < 1.4 us
void sample_callback( void )
{
int pos = etat.pos;
if	( pos < 0 )
	{				// pas de son en cours
	LL_TIM_OC_SetCompareCH1( TIM3, PWM_SILENCE );
	}
else if	( pos >= etat.tai )
	{				// son fini
	etat.pos = -1;
	}
else	{
	// extraire le code du buffer
	unsigned int pb0 = etat.pb0;
	unsigned int zecode = ( etat.zew >> pb0 );
	pb0 += QBIT;
	if	( pb0 >= 32 )
		{			// passer au word suivant
		etat.zew = etat.wbuf1[etat.iw++];
		pb0 -= 32;
		if	( pb0 > 0 )	// traiter residu
			zecode |= ( etat.zew << ( QBIT - pb0 ) ); 
		}
	etat.pb0 = pb0;
	zecode &= ( ( 1 << QBIT ) - 1 );
	// decoder le sample
	short sig = etat.oldsig;
	sig += dequant[zecode];
	etat.oldsig = sig;
	LL_TIM_OC_SetCompareCH1( TIM3, sig );
	// avancer le compteur
	pos++;
	etat.pos = pos;
	}
}

// leson est l'adresse d'un tableau dont le premier element est la taille du son
// la suite contient les codes entasses dans des mots de 32 bits
void audio_start( const unsigned int * leson )
{
etat.tai = leson[0];		// nombre de samples du son
etat.wbuf1 = leson + 1;		// pack de codes
etat.iw = 1;			// indice du word (32 bits) N.B. 1 parceque le premier word est lu ci-dessous
etat.pb0 = 0;			// position du lsb du code courant dans le word courant	
etat.zew = etat.wbuf1[0];	// word courant
etat.oldsig = PWM_SILENCE;	// predicteur N.B. cette valeur initiale determine la composante continue
etat.pos = 0;			// position en samples (-1 = stop)
}


void TIM3_IRQHandler(void)
{
static int cnt = 0;
if	( LL_TIM_IsActiveFlag_UPDATE( TIM3 ) )
	{
	P12_PROFIL_1();
	LL_TIM_ClearFlag_UPDATE( TIM3 );
	if	( ++cnt & 1 )		// en raison de l'oversampling X2, on doit interpoler
		sample_callback();	// interpolation grossiere (nearest neighbour)
	P12_PROFIL_0();
	}
}

