#include "stm32f1xx.h"
#include "flashy.h"

/* observations (pour STM32F03) :
	- l'effacement met des 11111111, classique
	- l'ecriture se fait par 16 bits, no choice
	- si on tente d'ecrire sur une position non effacee, l'ecriture echoue, sauf si on tente d'ecrire 0000
	  ==> on ne peut pas cumuler les zeros,
	  mais on peut marquer une position comme invalide pour economiser des erases
	- contrairement a ce que dit la doc du registre AR, on n'utilise pas ce registre pour les ecriture normales,
	  seulement pour effacer (on ecrit comme en RAM, halfword at a time)
   questions :
	- est-ce qu'il faut unlocker a chaque ecriture ? non
	  par contre il est safer de relocker apres usage

demo from lasergun:

		flashy_unlock();
		flashy_page_erase( LAST_FLASH_PAGE );	// derniere page
		short c = cheat_stat / 100;
		flashy_unlock();
		flashy_write_short( LAST_FLASH_PAGE,    c );
		flashy_unlock();
		flashy_write_short( LAST_FLASH_PAGE+2, ~c );

128 pages of 1 Kbyte (for medium-density devices),
de 08000000 a 08020000 : #define LAST_FLASH_PAGE (0x08020000-0x400)

*/

void flashy_unlock(void)
{
if	( FLASH->CR & FLASH_CR_LOCK )
	{
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
	}
}

void flashy_relock(void)
{
FLASH->CR |= FLASH_CR_LOCK;
}

void flashy_page_erase( unsigned int adr )
{
while	( FLASH->SR & FLASH_SR_BSY )
	{}
FLASH->CR = FLASH_CR_PER;
FLASH->AR = adr;
FLASH->CR |= FLASH_CR_STRT;
while	( FLASH->SR & FLASH_SR_BSY )
	{}
FLASH->CR = 0;		// sinon le bit PER reste
}

void flashy_write_short( unsigned int adr, unsigned short data )
{
while	( FLASH->SR & FLASH_SR_BSY )
	{}
FLASH->CR = FLASH_CR_PG;
/* Write data in the address */
*(__IO uint16_t*)adr = data;
while	( FLASH->SR & FLASH_SR_BSY )
	{}
}
