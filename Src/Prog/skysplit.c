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
*/
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

}

// ramener cap dans ] -PI/2, +PI/2 ]
float limit_cap( float c ) {
	//while	( c > Math.PI )
	while	( qfp_fsub( c, PI ) > 0.0f )
	//	c -= ( 2.0 * Math.PI );
		c = qfp_fsub( c, ( 2.0f * PI ) );
	//while	( c <= -Math.PI )
	while	( qfp_fadd( c, PI ) <= 0.0f )
	//	c += ( 2.0 * Math.PI );
		c = qfp_fadd( c, ( 2.0f * PI ) );
	return c;
	}
// notre convention:
//	head = cap en degres dans [ 0, 360 [
//	cap en radian dans ] -PI/2, +PI/2 ]
float head2cap( float h ) {
	// return limit_cap( Math.toRadians( 90.0 - h ) );
	return limit_cap( qfp_fmul( ToRadians, qfp_fsub( 90.0f, h ) ) );
	}
float cap2head( float c ) {
	//float h = 90.0 - Math.toDegrees( c );
	float h = qfp_fsub( 90.0f, qfp_fmul( ToDegrees, c ) );
	if	( h < 0.0f )
	//	h += 360.0;
		h = qfp_fadd( h, 360.0f );
	return h;
	}

void autopilot_init( autopilot *AP ) {
	AP->x = 0.0f;
        AP->y = 0.0f;
        AP->v = 0.1f;		// vitesse en Nm/s 0.1 <==> 360 knots
        AP->vx = AP->v;
        AP->vy = 0.0;
        AP->cap = 0.0;		// radian, repere trigo
        AP->w = 0.0;			// taux de virage en rad/s, signed
        AP->w3 = qfp_fmul( ToRadians, 3 );	// 3 deg/s
        AP->r3 = qfp_fdiv( AP->v, AP->w3 );		// rayon de virage pour 3 deg/s (1.9 NM @ 360 knots)
	}

// route depuis le point courant et le cap courant: virage puis segment
float angletoXY( autopilot *AP, float xb, float yb ) {
	#ifdef PROF_PB12
	PB12_PROFIL_1();
	#else
	DTICK_BEGIN();
	#endif
	// decider de quel cote tourner : cap approximatif
	//float capro = Math.atan2( yb - y, xb - x );
	float capro = qfp_fatan2( qfp_fsub( yb, AP->y ), qfp_fsub( xb, AP->x ) );
	//float dcapro = limit_cap( capro - cap );
	float dcapro = limit_cap( qfp_fsub( capro, AP->cap ) );
	//System.out.println( "capro=" + cap2head( capro ) + ", dcapro=" + cap2head( dcapro ) );
	// CDC_printf( "capro=%.5f, dcapro=%.5f\n", cap2head( capro ), qfp_fmul( ToDegrees, dcapro ) );
	// ici le signe de dcapro indique le sens du virage
	// chercher le centre C de l'arc de cercle
	float xc, yc;
	if	( dcapro > 0 )
		{	// C a gauche
	//	xc = x - r3 * Math.sin(cap);
		xc = qfp_fsub( AP->x, qfp_fmul( AP->r3, qfp_fsin( AP->cap ) ) );
	//	yc = y + r3 * Math.cos(cap);
		yc = qfp_fadd( AP->y, qfp_fmul( AP->r3, qfp_fcos( AP->cap ) ) );
		AP->w = AP->w3;
		}
	else	{	// C a droite
	//	xc = x + r3 * Math.sin(cap);
		xc = qfp_fadd( AP->x, qfp_fmul( AP->r3, qfp_fsin( AP->cap ) ) );
	//	yc = y - r3 * Math.cos(cap);
		yc = qfp_fsub( AP->y, qfp_fmul( AP->r3, qfp_fcos( AP->cap ) ) );
		AP->w = -AP->w3;
		}
	// System.out.println( "C " + xc + " " + yc );
	// CDC_printf( "C %.5f %.5f\n", xc, yc );
	// distance de C a B (B = destination finale)
	float dx, dy;
	//dx = xb - xc; dy = yb - yc;
	dx = qfp_fsub( xb, xc ); dy = qfp_fsub( yb, yc );
	//float modcb = Math.sqrt( dx * dx + dy * dy );
	float modcb = qfp_fsqrt( qfp_fadd( qfp_fmul( dx, dx ), qfp_fmul( dy, dy ) ) ); 
	//System.out.println( "|CB| " + modcb );
	// CDC_printf( "|CB| %.5f\n", modcb );
	if	( modcb < AP->r3 )	// si D est dans le cercle, on ne sait pas faire
	//	{ System.out.println("too close, giving up"); return; }
		{ CDC_printf("too close, giving up\n"); return AP->cap; }
	// angle CBD (D = fin virage), non signé et aigu
	//float cbd = Math.asin( r3 / modcb );
	float lesin = qfp_fdiv( AP->r3, modcb );
	float lecos = qfp_fsqrt( qfp_fsub( 1.0f, qfp_fmul( lesin, lesin ) ) );
	float cbd = qfp_fatan2( lesin, lecos );
	//System.out.println( "CBD " + Math.toDegrees(cbd) );
	// CDC_printf( "CBD %.5f\n", qfp_fmul( ToDegrees, cbd ) );
	// argument de CB
	//float argcb = Math.atan2( yb - yc, xb - xc );
	float argcb = qfp_fatan2( qfp_fsub( yb, yc ), qfp_fsub( xb, xc ) );
	//System.out.println( "arg CB " + cap2head(argcb) );
	// CDC_printf( "arg CB %.5f\n", cap2head(argcb) );
	// cap exact
	float cape;
	if	( dcapro > 0 )
	//	cape = argcb + cbd;
		cape = qfp_fadd( argcb, cbd );
	//else	cape = argcb - cbd;
	else	cape = qfp_fsub( argcb, cbd );
	//System.out.println( "cap exact " + cap2head(cape) );
	// CDC_printf( "cap exact %.5f\n", cap2head(cape) );
	#ifdef PROF_PB12
	PB12_PROFIL_0();
	#else
	DTICK_END();
	#endif
	return cape;
	}

void test_b() {
	autopilot APilot;
	autopilot_init( &APilot );

	APilot.x = 5.0f; APilot.y = 0.0f;				// @ 8 MHz	@ 72 MHz

	APilot.cap = angletoXY( &APilot, -5.0f, -1.0f );		// 1495cy 186u	2279cy 31.2u
	//CDC_printf( "cap exact %.5f\n", cap2head(APilot.cap) );
	APilot.x = -5.0f; APilot.y = -1.0f;

	APilot.cap = angletoXY( &APilot, 10.0f, -6.0f );		// 1855cy 230u	2811cy 38.6u
	//CDC_printf( "cap exact %.5f\n", cap2head(APilot.cap) );
	APilot.x = 10.0f; APilot.y = -6.0f;

	APilot.cap = angletoXY( &APilot, 10.0f, 5.0f );			// 1640cy 204u	2462cy 33.8u
	//CDC_printf( "cap exact %.5f\n", cap2head(APilot.cap) );
	APilot.x = 10.0f; APilot.y = 5.0f;

	APilot.cap = angletoXY( &APilot, 0.0f, 5.0f );			// 1618cy 201u	2435cy 33.4u
	//CDC_printf( "cap exact %.5f\n", cap2head(APilot.cap) );
	APilot.x = 0.0f; APilot.y = 5.0f;

	APilot.cap = angletoXY( &APilot, -30.0f, 0.0f );		// 1721cy 213u	2621cy 36.0u
	// attendu cap 260.53827
	CDC_printf( "cap exact %.5f\n", cap2head(APilot.cap) );
	#ifndef PROF_PB12
	CDC_printf( "dtick = %d\n", dtick );
	#endif
	}

