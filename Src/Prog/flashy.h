void flashy_unlock(void);
void flashy_relock(void);
void flashy_page_erase( unsigned int adr );
void flashy_write_short( unsigned int adr, unsigned short data );

// STM32F103
#define LAST_FLASH_PAGE (0x08020000-0x400)
