
///
/// zone LSB-LSB, inspiree du livre de Bentham (style zlib, Frevo, Green-Resort, KS)
///

// data byte is read LSB first
// register (*pcrc) read LSB first

// version pour MPU 8 bits
static void update_crc_ben8( unsigned int poly, unsigned int * pcrc, unsigned char byte )
{
char i; unsigned char lsb;
for	( i = 0; i < 8; i++ )
	{
	lsb = (unsigned char)*pcrc;		// optim for 8-bit machine, pour eviter un XOR sur 32 bits
	*pcrc >>= 1;
	if	( ( lsb ^ byte ) & 1 )
		*pcrc ^= poly;
	byte >>= 1;
	}
}

// version pour machine 32 bits, plus claire (meme resultat :-)
static void update_crc_ben32( unsigned int poly, unsigned int * pcrc, unsigned char byte )
{
unsigned int i, bit;
for	( i = 0; i < 8; i++ )
	{
	bit = ( *pcrc ^ byte ) & 1;
	*pcrc >>= 1;
	if	( bit )
		*pcrc ^= poly;
	byte >>= 1;
	}
}

unsigned int crc_ben( unsigned int init, unsigned int poly, unsigned int xorout, const unsigned char *buf, int len )
{
unsigned int crc = init;

do  {
    update_crc_ben32( poly, &crc, *(buf++) );
    } while (--len);

return crc ^ xorout;
}

///
/// zone MSB-MSB, (style reveng avec refin=false)
///

// data byte is read MSB first
// register (*pcrc) read MSB first

// version pour machine 32 bits
static void update_crc_mm32( unsigned int poly, unsigned int * pcrc, unsigned char byte )
{
unsigned int i, msbin, msbreg, bit;
for	( i = 0; i < 8; i++ )
	{
	msbin  = byte >> 7;
	msbreg = *pcrc >> 31;
	bit = ( msbin ^ msbreg ) & 1;
	*pcrc <<= 1;
	if	( bit )
		*pcrc ^= poly;
	byte <<= 1;
	}
}

unsigned int crc_mm( unsigned int init, unsigned int poly, unsigned int xorout, const unsigned char *buf, int len )
{
unsigned int crc = init;

do  {
    update_crc_mm32( poly, &crc, *(buf++) );
    } while (--len);

return crc ^ xorout;
}
