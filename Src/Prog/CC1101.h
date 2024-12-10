
// SPI1 en full duplex
void SPI1_init(void);

// ecrire et lire cnt bytes en une transaction
void SPI1_multi_byte( unsigned char * txbuf, unsigned char * rxbuf, int cnt );
