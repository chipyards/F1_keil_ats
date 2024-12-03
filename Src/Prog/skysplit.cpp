// simulation du pilote automatique: calcul de trajectoire
#include "qfplib-m3.h"
#include "options.h"
#include "skysplit.h"
#include "CDC.h"
#include "prof_tick.h"
#include <math.h>

// 2 outils de profilage
#ifdef PROF_PB12
#include "stm32f1xx_ll_gpio.h"
#include "gpio.h"
#else
DTICK_VARS
#endif

Apilot lepilot;

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
void Apilot::dump_loc() {
	CDC_printf( "L %.3f %.3f %.3f\n", x, y, cap2head(cap) );
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
		{
		//CDC_printf("too close, too close, going beyond\n");
		return 99.0f;
		}
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
// // methodes de simulation, iterent step()
// step nominalement d'une seconde (pour le moment)
void Apilot::step() {
	if	( ds )
		{
		while	( cnt100Hz < next_step )
			{ }
		next_step += ds;
		}
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
	dump_loc();
	}
// simple segment de droite de longueur d depuis le point courant x, y
// au cap courant
void Apilot::gotoD( float d ) {
	w = 0.0f;	// ligne droite
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
void Apilot::gotoXY( float xd, float yd ) {
	w = 0.0;	// ligne droite
	//float dx = xd - x;
	float dx = qfp_fsub( xd, x );
	//float dy = yd - y;
	float dy = qfp_fsub( yd, y );
	//float d = Math.sqrt( dx * dx + dy * dy );
	float d = qfp_fsqrt( qfp_fadd( qfp_fmul( dx, dx ), qfp_fmul( dy, dy ) ) );
	//int cnt = (int)Math.round( d / v );
	int cnt = (int)qfp_fadd( 0.5, qfp_fdiv( d, v ) );
	//cap = Math.atan2( dy, dx );
	cap = qfp_fatan2( dy, dx );
	//vx = v * Math.cos(cap);
	vx = qfp_fmul( v, qfp_fcos(cap) );
	//vy = v * Math.sin(cap);
	vy = qfp_fmul( v, qfp_fsin(cap) );
	for	( int i = 0; i < cnt; i++ )
		step();
	}
// arc de cercle depuis le point courant x, y et le cap courant
// sens automatique (virage < 180 deg)
void Apilot::turnTo( float cap2 ) {
	w = w3;
	float dc = limit_cap( cap2 - cap );
	//if	( dc < 0.0f )
	if	( jfp_fsgn( dc ) )
		w = -w;
	// int cnt = (int)Math.round( dc / w );
	int cnt = (int)qfp_fadd( 0.5, qfp_fdiv( dc, w ) );
	for	( int i = 0; i < cnt; i++ )
		step();
	}
// arc de cercle depuis le point courant x, y et le cap courant
// en imposant le taux (w) et le sens de rotation (signe de w)
void Apilot::turnTo( float cap2, float w ) {
	float dc = limit_cap( cap2 - cap );
	//if	( ( dc < 0.0 ) && ( w > 0.0 ) )
	if	( jfp_fsgn( dc ) && !jfp_fsgn( w ) )
	//	dc += ( 2 * Math.PI );
		dc = qfp_fadd( dc, qfp_fmul( 2.0f, PI ) );
	//else if	( ( dc > 0.0 ) && ( w < 0.0 ) )
	else if	( !jfp_fsgn( dc ) && jfp_fsgn( w ) )
	//	dc -= ( 2 * Math.PI );
		dc = qfp_fsub( dc, qfp_fmul( 2.0f, PI ) );
	//int cnt = Math.abs( (int)Math.round( dc / w ) );
	int cnt = abs( (int)qfp_fadd( 0.5, qfp_fdiv( dc, w ) ) );
	for	( int i = 0; i < cnt; i++ )
		step();
	}

// route depuis le point courant et le cap courant: virage puis segment, ou si trop pres,
// segment puis virage puis segment
void Apilot::routetoXY( float xb, float yb ) {
	float cape = angletoXY( xb, yb );
	if	( cape > 90.0f )
		{	// on va s'eloigner en ligne droite car le point vise est trop proche
		//gotoD( 2.0 * r3 );
		gotoD( qfp_fmul( 2.0f, r3 ) );	// avec 2r on est sur (mais c'est trop dans la plupart des cas)
		// CDC_printf( "diverted to "); dump_loc();
		// "nouveau calcul"
		cape = angletoXY( xb, yb );
		}
	// CDC_printf( "cap exact %.5f\n", cap2head(cape) );
	turnTo( cape, w );
	// dump_loc();
	gotoXY( xb, yb );
	// dump_loc();
	}



// cette demo est le portage du commit a69e2f6 de FXsim.java (https://gitlab.com/chipyards/ivy-rejeu.git)
// elle doit donner les memes resultats, avec une precicion degradee due au remplacement de float64 par float32
void Apilot::demo() {
	// CDC_printf( "rayon virage %.5f NM\n", r3 );
	next_step = cnt100Hz;
	gotoXY( 5.0, 0.0 );
	routetoXY( -5, -1 );
	routetoXY( 10, -6 );
	routetoXY( 10, 5 );
	routetoXY( 0, 5 );
	routetoXY( -30, 0 );
	routetoXY( -5, -10 );
	routetoXY( 10, -10 );
	routetoXY( 30, 0 );
	routetoXY( 0, 10 );
	routetoXY( -20, 10 );
	routetoXY( -15, 20 );
	routetoXY( -15, 10 );
	routetoXY( -10, 20 );
	routetoXY( -10, 10 );
	routetoXY( 0 , 24 );
	routetoXY( 24, 0 );
	routetoXY( 0, -24 );
	routetoXY( -24, 0 );
	routetoXY( 1.2, 2 );
	routetoXY( 0, 0 );
	routetoXY( -15, 0 );
	routetoXY( -10, 0 );
	routetoXY( -5, 0 );
	routetoXY( 0, 0 );
	}

void test_b() {
lepilot.demo();
}
