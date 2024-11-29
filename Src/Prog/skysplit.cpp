// simulation du pilote automatique: calcul de trajectoire
#include "qfplib-m3.h"
#include "options.h"
#include "skysplit.h"
#include "CDC.h"
#include "prof_tick.h"

// 2 outils de profilage
#ifdef PROF_PB12
#include "stm32f1xx_ll_gpio.h"
#include "gpio.h"
#else
DTICK_VARS
#endif

void test_a() {
/*
float a = qfp_fadd( 1.0f, PI );
float b = -a;
CDC_printf("-1-PI=%.8f\n", b );
int c;
c = ( b > 0.0f );
CDC_printf("%d \n", c );
c = ( b >= 0.0f );
CDC_printf("%d \n", c );
c = ( b < 0.0f );
CDC_printf("%d \n", c );
c = ( b <= 0.0f );
CDC_printf("%d \n", c );
c = ( b == 0.0f );
CDC_printf("%d \n", c );
c = ( b == a );
CDC_printf("%d \n", c );
c = ( b > a );
CDC_printf("%d \n", c );

float h;
h = 0.0; CDC_printf("%.7f rad -> head = %.7f deg\n", h, cap2head(h) );
h = 0.5 * PI; CDC_printf("%.7f rad -> head = %.7f deg\n", h, cap2head(h) );
h = PI; CDC_printf("%.7f rad -> head = %.7f deg\n", h, cap2head(h) );
h = 1.5 * PI; CDC_printf("%.7f rad -> head = %.7f deg\n", h, cap2head(h) );
h = 2.0 * PI; CDC_printf("%.7f rad -> head = %.7f deg\n", h, cap2head(h) );
h = -0.5 * PI; CDC_printf("%.7f rad -> head = %.7f deg\n", h, cap2head(h) );
h = -PI; CDC_printf("%.7f rad -> head = %.7f deg\n", h, cap2head(h) );
h = -1.5 * PI; CDC_printf("%.7f rad -> head = %.7f deg\n", h, cap2head(h) );
h = -2.0 * PI; CDC_printf("%.7f rad -> head = %.7f deg\n", h, cap2head(h) );
float d;
d = 0.0f; CDC_printf("%.2f deg -> cap = %.7f rad\n", d, head2cap(d) );
d = 90.0f; CDC_printf("%.2f deg -> cap = %.7f rad\n", d, head2cap(d) );
d = 180.0f; CDC_printf("%.2f deg -> cap = %.7f rad\n", d, head2cap(d) );
d = 270.0f; CDC_printf("%.2f deg -> cap = %.7f rad\n", d, head2cap(d) );
d = 360.0f; CDC_printf("%.2f deg -> cap = %.7f rad\n", d, head2cap(d) );
d = 450.0f; CDC_printf("%.2f deg -> cap = %.7f rad\n", d, head2cap(d) );
d = -90.0f; CDC_printf("%.2f deg -> cap = %.7f rad\n", d, head2cap(d) );
d = -180.0f; CDC_printf("%.2f deg -> cap = %.7f rad\n", d, head2cap(d) );
d = -270.0f; CDC_printf("%.2f deg -> cap = %.7f rad\n", d, head2cap(d) );

float z = -0.000001f;
CDC_printf("signe de %.7f = 0x%08x\n", z, jfp_fsgn(z) );
z = jfp_fabs( z );
CDC_printf("signe de %.7f = 0x%08x\n", z, jfp_fsgn(z) );
float x = qfp_fsqrt( z );			// rend nan si arg negatif
CDC_printf("racine de %.7f = %.7f\n", z, x );
// float __ieee754_sqrtf( float );
// x = __ieee754_sqrtf( z );			// runtime gcc, rend nan si arg negatif
// CDC_printf("racine de %.7f = %.7f\n", z, x );
*/

float z; int i;
// rounding positive float
z = 1.0f;     i = (int)qfp_fadd( 0.5, z ); CDC_printf("%.5f -> %d\n", z, i );
z = 0.51f;    i = (int)qfp_fadd( 0.5, z ); CDC_printf("%.5f -> %d\n", z, i );
z = 0.49f;    i = (int)qfp_fadd( 0.5, z ); CDC_printf("%.5f -> %d\n", z, i );
z = 20.99f;   i = (int)qfp_fadd( 0.5, z ); CDC_printf("%.5f -> %d\n", z, i );
z = 777.501f; i = (int)qfp_fadd( 0.5, z ); CDC_printf("%.5f -> %d\n", z, i );
z = 66.499f;  i = (int)qfp_fadd( 0.5, z ); CDC_printf("%.5f -> %d\n", z, i );
z = 1111.0f;  i = (int)qfp_fadd( 0.5, z ); CDC_printf("%.5f -> %d\n", z, i );
// rounding negative float
z = -3.1f;    i = (int)qfp_fsub( 0.5, z ); CDC_printf("%.5f -> %d\n", z, i );
z = -5.51f;   i = (int)qfp_fsub( 0.5, z ); CDC_printf("%.5f -> %d\n", z, i );

}


// // methodes de calcul
// ramener cap dans ] -PI/2, +PI/2 ]
float Apilot::limit_cap( float c ) {
	//while	( jfp_fsgn( qfp_fsub( PI, c ) ) )	// faster, inaccurate
	while	( c > PI )
		c = qfp_fsub( c, ( 2.0f * PI ) );
	//while	( jfp_fsgn( qfp_fadd( c, PI ) ) )	// faster, inaccurate
	while	( c <= -PI )
		c = qfp_fadd( c, ( 2.0f * PI ) );
	return c;
	}
// notre convention:
//	head = cap en degres dans [ 0, 360 [
//	cap en radian dans ] -PI/2, +PI/2 ]
float Apilot::head2cap( float h ) {
	return limit_cap( qfp_fmul( ToRadians, qfp_fsub( 90.0f, h ) ) );
	}
float Apilot::cap2head( float c ) {
	float h = qfp_fsub( 90.0f, qfp_fmul( ToDegrees, c ) );
	//if	( jfp_fsgn(h) )				// faster, inaccurate
	if	( h < 0.0f )
		h = qfp_fadd( h, 360.0f );
	return h;
	}
// preparation de la route depuis le point courant et le cap courant: virage puis segment
// cette methode calcule le cap destination de ce virage
float Apilot::angletoXY( float xb, float yb ) {
	// decider de quel cote tourner : cap approximatif
	float capro = qfp_fatan2( qfp_fsub( yb, y ), qfp_fsub( xb, x ) );
	float dcapro = limit_cap( qfp_fsub( capro, cap ) );
	//CDC_printf( "capro=%.5f, dcapro=%.5f\n", cap2head( capro ), qfp_fmul( ToDegrees, dcapro ) );
	// ici le signe de dcapro indique le sens du virage
	// chercher le centre C de l'arc de cercle
	float xc, yc;
	if	( jfp_fsgn(dcapro) )
		{	// C a droite
		xc = qfp_fadd( x, qfp_fmul( r3, qfp_fsin( cap ) ) );
		yc = qfp_fsub( y, qfp_fmul( r3, qfp_fcos( cap ) ) );
		w = -w3;
		}
	else	{	// C a gauche
		xc = qfp_fsub( x, qfp_fmul( r3, qfp_fsin( cap ) ) );
		yc = qfp_fadd( y, qfp_fmul( r3, qfp_fcos( cap ) ) );
		w = w3;
		}
	//CDC_printf( "C %.5f %.5f\n", xc, yc );
	// distance de C a B (B = destination finale)
	float dx, dy;
	dx = qfp_fsub( xb, xc ); dy = qfp_fsub( yb, yc );
	float modcb = qfp_fsqrt( qfp_fadd( qfp_fmul( dx, dx ), qfp_fmul( dy, dy ) ) ); 
	//CDC_printf( "|CB| %.5f\n", modcb );
	if	( jfp_fsgn( qfp_fsub( modcb, r3 ) ) )	// si D est dans le cercle, on ne sait pas faire
		{ CDC_printf("too close, too close, going beyond\n"); return 99.0f; }
	// angle CBD (D = fin virage), non signé et aigu
	float lesin = qfp_fdiv( r3, modcb );
	float lecos = qfp_fsqrt( jfp_fabs( qfp_fsub( 1.0f, qfp_fmul( lesin, lesin ) ) ) );
	float cbd = qfp_fatan2( lesin, lecos );
	//CDC_printf( "CBD %.5f\n", qfp_fmul( ToDegrees, cbd ) );
	// argument de CB
	float argcb = qfp_fatan2( qfp_fsub( yb, yc ), qfp_fsub( xb, xc ) );
	//CDC_printf( "arg CB %.5f\n", cap2head(argcb) );
	// cap exact
	float cape;
	if	( jfp_fsgn(dcapro) )
		cape = qfp_fsub( argcb, cbd );
	else	cape = qfp_fadd( argcb, cbd );
	//CDC_printf( "cap exact %.5f\n", cap2head(cape) );
	return cape;
	}
/*
// // methodes de simulation, iterent step()
// step d'une seconde (pour le moment)
void step() {
	if	( w != 0.0f )
		{
		//cap += w;
		cap = qfp_fadd( cap, w );
		//vx = v * Math.cos(cap);
		vx = qfp_fmul( v, qfp_fcos(cap) );
		//vy = v * Math.sin(cap);
		vy = qfp_fmul( v, qfp_fsin(cap) );
		}
	//x += vx;
	x = qfp_fadd( x, vx );
	//y += vy;
	y = qfp_fadd( y, vy );
	// track.add( new Punkt( x, y ) );
	}
// simple segment de droite de longueur d depuis le point courant x, y
// au cap courant
void gotoD( double d ) {
	w = 0.0;	// ligne droite
	//int cnt = (int)Math.round( d / v );
	int cnt = (int)qfp_fadd( 0.5, qfp_fdiv( d, v ) );
	//vx = v * Math.cos(cap);
	vx = qfp_fmul( v, qfp_fcos(cap) );	
	//vy = v * Math.sin(cap);
	vy = qfp_fmul( v, qfp_fsin(cap) );
	for	( int i = 0; i < cnt; i++ )
		step();
	}
// simple segment de droite depuis le point courant x, y (recalcule le cap)
void gotoXY( double xd, double yd ) {
	w = 0.0;	// ligne droite
	double dx = xd - x;
	double dy = yd - y;
	double d = Math.sqrt( dx * dx + dy * dy );
	int cnt = (int)Math.round( d / v );
	cap = Math.atan2( dy, dx );
	vx = v * Math.cos(cap);
	vy = v * Math.sin(cap);
	for	( int i = 0; i < cnt; i++ )
		step();
	}
// arc de cercle depuis le point courant x, y et le cap courant
// sens automatique (virage < 180 deg)
void turnTo( double cap2 ) {
	w = w3;
	double dc = limit_cap( cap2 - cap );
	if	( dc < 0.0 )
		w = -w;
	int cnt = (int)Math.round( dc / w );
	for	( int i = 0; i < cnt; i++ )
		step();
	}
*/



void Apilot::demo() {

	x = 5.0f; y = 0.0f;				// previous code	this code (both @ 8MHz)

	cap = angletoXY( -5.0f, -1.0f );		// 1495cy 186u		1264cy 156u
	//CDC_printf( "cap exact %.5f\n", cap2head(cap) );
	x = -5.0f; y = -1.0f;

	cap = angletoXY( 10.0f, -6.0f );		// 1855cy 230u		1588cy 197u
	//CDC_printf( "cap exact %.5f\n", cap2head(cap) );
	x = 10.0f; y = -6.0f;

	cap = angletoXY( 10.0f, 5.0f );			// 1640cy 204u		1436cy 179u
	//CDC_printf( "cap exact %.5f\n", cap2head(cap) );
	x = 10.0f; y = 5.0f;

	cap = angletoXY( 0.0f, 5.0f );			// 1618cy 201u		1410cy 175u
	//CDC_printf( "cap exact %.5f\n", cap2head(cap) );
	x = 0.0f; y = 5.0f;

	cap = angletoXY( -30.0f, 0.0f );		// 1721cy 213u		1461cy 181u
	// attendu cap 260.53827
	CDC_printf( "cap exact %.5f\n", cap2head(cap) );
	}

void test_b() {
Apilot lepilot;

lepilot.demo();
}
