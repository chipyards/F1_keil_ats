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

// ROM data
const float rom_beacons[] = {
	0, 0,		// 0  Z
	0, 24,		// 1  N
	24, 0,		// 2  E
	0, -24,		// 3  S
	-24, 0,		// 4  O
	-15, 20,	// 5  A
	-15, 10,	// 6  B
	-10, 20,	// 7  C
	-10, 10,	// 8  D
	10, 20,		// 9  E
	10, 10		// 10 F
	};

void test_a() {
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
		return 666.6f;
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
// step, nominalement d'une seconde (pour le moment)
// fonction a iterer depuis la boucle principale
void Apilot::step() {
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
	// ici on doit tester s'il n'y a pas une requete de diversion
	if	( diversion >= 0 )
		{				// diversion vers un autre waypoint
		target_waypoint = diversion;
		const Beacon *b = get_beacon( target_waypoint );
		if	( b )
			routetoXY( b->x, b->y );
		else	adrift();
		diversion = -1;		// acknowledge
		return;
		}
	if	( diversion == -2 )
		{				// abandon du plan en cours
		adrift();
		return;
		}
	if	( diversion == -3 )
		{				// virage de diversion au cap demande puis ligne droite
		turnTo( cap_diversion );
		segtype = 4;
		return;
		}
	if	( cnt > 1 )		// il reste au moins 1 point, continuer le segment
		{ cnt--; return; }
	// ici on va preparer un nouveau segment
	switch	( segtype )
		{
		case 0:	{	// fin de la derniere droite d'une "route to XY" : on a atteint le waypoint vise
			target_waypoint = get_next_waypoint();
			if	( target_waypoint < 0 )
				{ adrift(); return; }
			const Beacon *b = get_beacon( target_waypoint );
			if	( b )
				routetoXY( b->x, b->y );
			else	adrift();
			} break;
		case 1: {	// fin premier virage d'une "route to XY"
			const Beacon *b = get_beacon( target_waypoint );
			if	( b )
				{ gotoXY( b->x, b->y ); segtype = 0; }
			else	adrift();
			} break;
		case 3: {	// fin droite initiale (rare) d'une "route to XY"
			const Beacon *b = get_beacon( target_waypoint );
			if	( b )
				routetoXY( b->x, b->y );	// "nouveau calcul"
			else	adrift();
			} break;
		case 4: {	// fin virage de diversion
			adrift();
			} break;
		default : adrift();
		}
	} // step

// // methodes configurant la FSM pour les prochains steps
// fuir tout droit en attendant un ordre
void Apilot::adrift() {
	cnt = -1;	// indefini, au sens de sans limite imposee
	w = 0.0f;
	vx = qfp_fmul( v, qfp_fcos(cap) );
	vy = qfp_fmul( v, qfp_fsin(cap) );
	segtype = 0;
	target_waypoint = -2;
	}
// simple segment de droite de longueur d depuis le point courant x, y
// au cap courant
void Apilot::gotoD( float d ) {
	w = 0.0f;	// ligne droite
	cnt = (int)qfp_fadd( 0.5, qfp_fdiv( d, v ) );
	vx = qfp_fmul( v, qfp_fcos(cap) );	
	vy = qfp_fmul( v, qfp_fsin(cap) );
	}
// simple segment de droite depuis le point courant x, y (recalcule le cap)
void Apilot::gotoXY( float xd, float yd ) {
	w = 0.0f;	// ligne droite
	float dx = qfp_fsub( xd, x );
	float dy = qfp_fsub( yd, y );
	float d = qfp_fsqrt( qfp_fadd( qfp_fmul( dx, dx ), qfp_fmul( dy, dy ) ) );
	cnt = (int)qfp_fadd( 0.5, qfp_fdiv( d, v ) );
	cap = qfp_fatan2( dy, dx );
	vx = qfp_fmul( v, qfp_fcos(cap) );
	vy = qfp_fmul( v, qfp_fsin(cap) );
	}
// arc de cercle depuis le point courant x, y et le cap courant
// sens automatique (virage < 180 deg)
void Apilot::turnTo( float cap2 ) {
	w = w3;
	float dc = limit_cap( cap2 - cap );
	if	( jfp_fsgn( dc ) )
		w = -w;
	cnt = (int)qfp_fadd( 0.5, qfp_fdiv( dc, w ) );
	}
// arc de cercle depuis le point courant x, y et le cap courant
// en imposant le taux (w) et le sens de rotation (signe de w)
void Apilot::turnTo( float cap2, float w ) {
	float dc = limit_cap( cap2 - cap );
	if	( jfp_fsgn( dc ) && !jfp_fsgn( w ) )
		dc = qfp_fadd( dc, qfp_fmul( 2.0f, PI ) );
	else if	( !jfp_fsgn( dc ) && jfp_fsgn( w ) )
		dc = qfp_fsub( dc, qfp_fmul( 2.0f, PI ) );
	cnt = abs( (int)qfp_fadd( 0.5, qfp_fdiv( dc, w ) ) );
	}

// route depuis le point courant et le cap courant: virage puis segment, ou si trop pres,
// segment puis virage puis segment
void Apilot::routetoXY( float xb, float yb ) {
	float cape = angletoXY( xb, yb );
	// CDC_printf( "cap exact %.5f\n", cap2head(cape) );
	if	( cape > 666.0f )
		{	// on va s'eloigner en ligne droite car le point vise est trop proche
		gotoD( qfp_fmul( 2.0f, r3 ) );	// avec 2r on est sur (mais c'est trop dans la plupart des cas)
		segtype = 3;
		}
	else	{
		turnTo( cape, w );	// w a ete mis a jour par effet de bord de angletoXY, c'est pas clean
		segtype = 1;
		}
	}


void test_b() {
}
