// simulation du pilote automatique: calcul de trajectoire
#include "qfplib-m3.h"
#include "options.h"
#include "apilot.h"
#include "CDC.h"
#include "CC1101.h"
#include <math.h>
#include <stdio.h>	// pour snprintf
#include <string.h>	// pour strlen

// 2 outils de profilage *
#ifdef PROF_PB12
#include "stm32f1xx_ll_gpio.h"
#include "gpio.h"
#endif

#ifdef PROF_DTICK
#include "prof_tick.h"
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
	10, 20,		// 9  F
	10, 10		// 10 G
	};

void Apilot::load_plan() {	// chargement du plan par defaut
	int i = 0;
	/*
	plan[i++] = 1;	plan[i++] = 2;	plan[i++] = 3;	plan[i++] = 4;
	plan[i++] = 5;	plan[i++] = 6;	plan[i++] = 7;	plan[i++] = 8;	plan[i++] = 9;	plan[i++] = 10;	plan[i++] = 0;
	*/
	plan[i++] = 8;	plan[i++] = 7;	plan[i++] = 6;	plan[i++] = 5;
	qplan = i;
	}

void Apilot::init() {
	sim_speed = 1;
	t = 0;
	beacons = (Beacon *)rom_beacons;
        load_plan();
	// rates
        w3 = qfp_fmul( ToRadians, 3.0f );	// 3 deg/s = 0,05236 rd/s
        r3 = qfp_fdiv( v, w3 );			// rayon de virage pour 3 deg/s (1.9 NM @ 360 knots)
        // N.B. acceleration centrifuge : gamma = (v*v)/r = r*(w*w) = v * w ( 9.697 m/s2 @ 360 knots & 3 deg/s )
        // bank angle : b = atan2( gamma, g ) ( 44.6 deg  @ 360 knots & 3 deg/s ) ( passenger acft: normal is 33deg )
        // load factor : lf = 1/cos(b)
        v = 0.1f;		// vitesse en Nm/s 0.1 <==> 360 knots <==> 185.2 m/s
	fl = 220;
	// coordonnees de depart, au premier waypoint du plan si possible
	if	( qplan >= 1 )
		{
		curway = plan[0];
		const Beacon *b = get_beacon( curway );
		if	( b )
			{ x = b->x; y = b->y; }
		else	{ x = 0.0f; y = 0.0f; }
		}
	else	{ x = 0.0f; y = 0.0f; }
	// route initiale, vers second point du plan si possible
	if	( qplan >= 2 )
		{
		iplan = 1; curway = plan[iplan];
		const Beacon *b = get_beacon( curway );
		if	( b )
			routetoXY( b->x, b->y );
		else	{ vx = v; vy = 0.0f; cap = 0.0f; w = 0.0f; adrift(); }
		}
	else	{ vx = v; vy = 0.0f; cap = 0.0f; w = 0.0f; adrift(); }
	// temporaires
	diversion = -1;
	cap_diversion = 0.0f;
	};

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
	#ifdef USE_CDC
	CDC_printf( "L %6.2f %6.2f %6.2f div:%d cur:%d iplan:%d->%d seg:%d cnt:%d\n", x, y, cap2head(cap), diversion, curway, iplan, plan[iplan], segtype, cnt );
	#endif
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
	t++;
	// ici on doit tester s'il n'y a pas une requete de diversion, avant de tester cnt
	// possiblement la diversion va reinitialiser cnt et calculer une nouvelle route
	if	( diversion >= 0 )
		{				// diversion vers un autre waypoint
		curway = diversion;
		// si la diversion est dans le plan, iplan va permttre la continuation de ce plan
		// sinon iplan = -2 va faire quitter le plan apres cette diversion
		iplan = find_in_plan( curway );
		const Beacon *b = get_beacon( curway );
		if	( b )
			{
			routetoXY( b->x, b->y );
			//CDC_printf("diversion vers %d\n", curway );
			}
		else	adrift();
		diversion = -1;		// acknowledge
		return;
		}
//	if	( diversion == -2 )
//		{				// abandon du plan en cours
//		adrift();
//		diversion = -1;		// acknowledge
//		return;
//		}
	if	( diversion == -3 )
		{				// virage de diversion au cap demande puis ligne droite
		turnTo( cap_diversion );
		segtype = 4;
		diversion = -1;		// acknowledge
		return;
		}
	if	( cnt > 1 )		// il reste au moins 1 point, continuer le segment
		{ cnt--; return; }
	// diversion ou pas, le segment est fini, on va preparer un nouveau segment
	switch	( segtype )
		{
		case 0:	{	// fin de la derniere droite d'une "route to XY" : on a atteint le waypoint vise
			if	( iplan <= -2 )		// on vient d'une diversion hors plan
				{ adrift(); return; }	// le plan est fini
			iplan++;
			if	( iplan >= int(qplan) )			// on a atteint le bout du plan
				{ iplan = -2; adrift(); return; }	// le plan est fini
			// ici on sait ou aller
			curway = plan[iplan];
			const Beacon *b = get_beacon( curway );
			if	( b )
				routetoXY( b->x, b->y );
			else	adrift();
			} break;
		case 1: {	// fin premier virage d'une "route to XY"
			const Beacon *b = get_beacon( curway );
			if	( b )
				{ gotoXY( b->x, b->y ); segtype = 0; }
			else	adrift();
			} break;
		case 3: {	// fin droite initiale (rare) d'une "route to XY"
			const Beacon *b = get_beacon( curway );
			if	( b )
				routetoXY( b->x, b->y );	// "nouveau calcul"
			else	adrift();
			} break;
		case 4: {	// fin virage de diversion
			adrift();
			} break;
		case 5:		// drifting : nothing to do
			break;
		default : adrift();
		}
	} // step

// // methodes configurant la FSM pour les prochains steps
// fuir tout droit en attendant un ordre
void Apilot::adrift() {
	cnt = -1;	// indefini, au sens de sans limite imposee
	diversion = -1;	// pas de diversion
	w = 0.0f;
	vx = qfp_fmul( v, qfp_fcos(cap) );
	vy = qfp_fmul( v, qfp_fsin(cap) );
	segtype = 5;
	curway = -2;
	iplan = -2;
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
	//CDC_printf( "cap exact %.2f\n", cap2head(cape) );
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

// verification de CRC32 dans packet p (le crc est a p + (p[0]-3))
int Apilot::CRC32ok( unsigned char * p )
{
return 1;
}

// interpreteur de commandes (paquet radio, 1er byte is LEN, ADR verified)
void Apilot::cmd_handler( unsigned char * p )
{
switch	( opcode_t(p[2]) )
	{
	// pilot orders
	case NEWFP:  if ( ( p[0] >= 7 ) && ( CRC32ok(p) ) )
			{
			qplan = p[0] - 6;	// len - {adr, opcode, crc}
			if	( qplan < 55 ) qplan = 55;
			for	( unsigned int i = 0; i < qplan; i++ )
				{
				if	( p[i+3] < QBEACON )
					plan[i] = p[i+3];
				else	{ queue_unable( BADWAY, p + (p[0]-3) ); return; }
				}
			unsigned int nextway = plan[0];
			lepilot.diversion = (int)nextway;
			queue_wilco( p + (p[0]-3) );
			}
		break;
	case DIRECT: if ( ( p[0] == 7 ) && ( CRC32ok(p) ) )
			{
			unsigned int wpt = p[3];
			if	( wpt >= QBEACON )
				{ queue_unable( BADWAY, p+4 ); return; }
			lepilot.diversion = (int)wpt;
			queue_wilco( p+4 );
			}
		break;
	case TURN:   if ( ( p[0] == 8 ) && ( CRC32ok(p) ) )
			{
			int newhead = (int)from_u16le( p+3 );
			if	( ( newhead < -180 ) || ( newhead > 360 ) )
				{ queue_unable( BADHDG, p+4 ); return; }
			lepilot.diversion = -3;
			lepilot.cap_diversion = lepilot.head2cap(float(newhead));
			//CDC_printf("cap_d = %d = %.3f\n", newhead, lepilot.cap_diversion );
			queue_wilco( p+5 );
			}
		break;
	case NEWFL:  if ( ( p[0] == 8 ) && ( CRC32ok(p) ) )
			{
			unsigned int newfl = from_u16le( p+3 );
			if	( ( newfl < FLMIN ) || ( newfl > FLMAX ) )
				{ queue_unable( BADFL, p+4 ); return; }
			queue_wilco( p+5 );
			}
		break;
	// simulation commands
	case PAUSE:  if ( p[0] == 2 ) CC.AAR_tx_enable = 0;
		break;
	case RESUME: if ( p[0] == 2 )  CC.AAR_tx_enable = 1;
		break;
	case RATECK: if ( p[0] == 3 )
			{
			sim_speed = p[3];
			if	( sim_speed > 8 )
				sim_speed = 8;
			}
		break;
	case SRESET: if ( p[0] == 2 ) init();
		break;
	default: ;
	}
}

// navigation automatic report, including navigation steps
int Apilot::AAR_tx()
{
for	( unsigned int i = 0; i < sim_speed; i++ )
	{
	#ifdef USE_CDC
	dump_loc();
	#endif
	#ifdef PROF_DTICK
	DTICK_BEGIN();
	#endif
	#ifdef PROF_PB12
	PB12_PROFIL_1();
	#endif
	step();	// calcule la position et tout
	#ifdef PROF_PB12
	PB12_PROFIL_0();
	#endif
	#ifdef PROF_DTICK
	DTICK_END();
	CDC_printf("* %d < %d\n", dtick, max_dtick );
	#endif
	}
unsigned char ubuf[16];
ubuf[0] = 14;
ubuf[1] = FLIGHT | 0x80;
ubuf[2] = opcode_t(VAAR);
to_s16le( ubuf+3, qfp_fmul( x, 100.0f ) );
to_s16le( ubuf+5, qfp_fmul( y, 100.0f ) );
to_s16le( ubuf+7, qfp_fmul( vx, 36000.0f ) );	// convert Nm/s to knots*10
to_s16le( ubuf+9, qfp_fmul( vy, 36000.0f ) );
to_u16le( ubuf+11, fl );
to_u16le( ubuf+13, t );
return CC.tx_if_can( ubuf );
}

// mise en queue d'un WILCO (revoie les 4 bytes du crc)
void Apilot::queue_wilco( unsigned char * crcbuf )
{
}

// mise en queue d'un UNABLE (revoie le byte d'err code suivi des 4 bytes de CRC)
void Apilot::queue_unable( err_t err, unsigned char * crcbuf )
{
}

