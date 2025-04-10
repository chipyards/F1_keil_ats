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

// source beacons_sorted_traffic_small.txt trie par beacons_bordeaux_toulon.html
const short rom_beacons[] = {
715,-1475,1976,-1583,1898,-1617,2035,-1728,703,-1667,2047,-1737,995,-1538,228,-1368,
299,-1497,1466,-892,1873,-1619,2128,-1552,1523,-865,1087,-1203,897,-1508,1643,-882,
2007,-727,195,-2064,805,-972,1654,-775,-351,-934,346,-1919,1499,-1184,195,-1608,
568,-1252,1646,-1392,1094,-1381,1574,-1656,1773,-1866,1031,-1369,-511,-1892,664,-952,
208,-1051,-396,-1721,2139,-1657,848,-1399,-348,-1558,-249,-1041,2202,-1252,1160,-1739,
1674,-1800,481,-990,1048,-1541,-528,-1695,327,-958,-490,-1696,1875,-1811,44,-792,
582,-1756,486,-1254,1701,-1437,773,-1809,2152,-1729,771,-1608,1430,-1375,992,-1529,
747,-1378,2113,-1360,-155,-1176,1683,-1847,-519,-1782,-235,-1343,1191,-1295,1220,-1342,
973,-1207,1433,-1639,-292,-863,1572,-828,1798,-1161,707,-1571,423,-1216,1382,-1608,
635,-1457,614,-1789,2126,-1620,222,-1920,1231,-1582,2137,-1624,792,-1127,1886,-1348,
354,-2034,1854,-724,980,-1207,266,-1699,404,-1558,1984,-1847,1006,-1826,2285,-705,
853,-1783,939,-1521,1948,-1083,1833,-1814,1677,-1039,-285,-1170,310,-1295,413,-1207,
1098,-2044,-296,-1280,998,-1706,1911,-1057,1440,-908,-230,-1977,1168,-1488,1272,-810,
1097,-1467,2070,-1335,1213,-1174,-61,-1018,1804,-1527,659,-1745,2188,-1654,1766,-937,
-286,-1247,1305,-1299,1446,-1557,-346,-1938,2115,-1476,-418,-1557,417,-799,1075,-732,
1046,-1599,-294,-1098,95,-754,1562,-1695,787,-1689,2111,-1650,1706,-1518,1901,-2099,
698,-1475,2053,-1485,2154,-1762,1270,-1676,1084,-1131,1430,-918,1128,-1451,1376,-863,
1074,-891,113,-1099,1077,-963,1562,-1373,666,-1754,828,-1880,1958,-1622,1867,-1677,
1779,-1682,1641,-1127,1483,-761,1413,-1256,603,-1290,1975,-850,1048,-1541,1567,-862,
540,-1411,1529,-1471,1423,-1789,230,-729,1987,-1741,4,-1844,1788,-1113,2058,-1654,
2254,-1118,968,-1049,879,-1821,1676,-1318,-249,-1747,1836,-1857,1867,-761,305,-1400,
1847,-1640,1871,-1749,2091,-1864,53,-2050,326,-906,2108,-1288,1357,-744,1876,-896,
-388,-1913,207,-907,1869,-1715,1012,-2023,831,-1880,-105,-1772,309,-882,506,-1061,
1931,-1191,531,-917,1699,-1687,680,-1874,2057,-1736,-351,-1472,2243,-815,691,-2076,
1647,-833,1754,-856,1940,-1874,522,-1556,1728,-1710,2251,-1731,313,-1199,1659,-1738,
824,-1741,-51,-1114,1771,-1442,45,-1114,175,-1233,1029,-1806,1321,-1501,1825,-1858,
-422,-1173,990,-726,349,-1342,-445,-1533,1775,-2078,-88,-2026,376,-1558,402,-1678,
1621,-1860,891,-1124,-3,-1700,53,-1762,196,-1714,-342,-1030,-548,-1795,2190,-1629,
1881,-2004,2247,-1591,2166,-1824,457,-1590,2006,-1653,517,-1676,511,-773,2032,-1873,
2110,-1732,-367,-862,1078,-804,-112,-912,1188,-1569,1506,-1640,51,-970,1418,-822,
1730,-1575,1150,-1739,
}; // 242

const unsigned char rom_plan[] = { 8, 167, 210, 94, 198, 70, 24, 148 };

void Apilot::load_plan() {	// chargement du plan par defaut
	for	( unsigned int i = 0; i < sizeof(rom_plan); i++ )
		plan[i] = rom_plan[i];
	qplan = sizeof(rom_plan);
	}

void Apilot::init() {
	sim_speed = 1;
	t = 0;
	beacons = rom_beacons;
        load_plan();
	// rates
        v = 0.1f;	// vitesse en Nm/s 0.1 <==> 360 knots <==> 185.2 m/s
        w3 = qfp_fmul( ToRadians, 3.0f );	// 3 deg/s = 0,05236 rd/s
        r3 = qfp_fdiv( v, w3 );			// rayon de virage pour 3 deg/s (1.9 NM @ 360 knots)
        // N.B. acceleration centrifuge : gamma = (v*v)/r = r*(w*w) = v * w ( 9.697 m/s2 @ 360 knots & 3 deg/s )
        // bank angle : b = atan2( gamma, g ) ( 44.6 deg  @ 360 knots & 3 deg/s ) ( passenger acft: normal is 33deg )
        // load factor : lf = 1/cos(b)
        vz = 0.0f;
	vzup  =  0.30f;	// taux de montee, FL units/s
	vzdown = -0.40f;	// taux de descente, FL units/s
	fl = 220.0f;
	// coordonnees de depart, au premier waypoint du plan si possible
	if	( qplan >= 1 )
		{
		curway = plan[0];
		Beacon b;
		if	( get_beacon( &b, curway ) == 0 )
			{ x = b.x;  y = b.y;  }
		else	{ x = 0.0f; y = 0.0f; }
		}
	else	{ x = 0.0f; y = 0.0f; }
	// route initiale, vers second point du plan si possible
	if	( qplan >= 2 )
		{
		iplan = 1; curway = plan[iplan];
		Beacon b;
		if	( get_beacon( &b, curway ) == 0 )
			routetoXY( b.x, b.y );
		else	{ vx = v; vy = 0.0f; cap = 0.0f; w = 0.0f; adrift(); }
		}
	else	{ vx = v; vy = 0.0f; cap = 0.0f; w = 0.0f; adrift(); }
	// temporaires
	diversion = -1;
	cap_diversion = 0.0f;
	fl_request = 0.0f;
	};

// // accesseurs
int Apilot::get_beacon( Beacon * b, int i ) {
	if	( i < (int)QBEACON )
		{
		b->x = qfp_fmul( 0.125f, (float)beacons[i*2] );
		b->y = qfp_fmul( 0.125f, (float)beacons[1+i*2] );
		return 0;
		}
	return 1;
	};

// // methodes de calcul
// ramener cap dans ] -PI/2, +PI/2 ]
float Apilot::limit_cap( float c ) {
	//while	( jfp_fsgn( qfp_fsub( PI, c ) ) )	// faster, inaccurate
	while	( c > PI )
		c = qfp_fsub( c, PIx2 );
	//while	( jfp_fsgn( qfp_fadd( c, PI ) ) )	// faster, inaccurate
	while	( c <= -PI )
		c = qfp_fadd( c, PIx2 );
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
	if	( vz > 0.0f )
		{
		fl = qfp_fadd( fl, vz );
		if	( fl >= fl_request )
			{ fl = fl_request; vz = 0.0f; }
		}
	else if	( vz < 0.0f )
		{
		fl = qfp_fadd( fl, vz );
		if	( fl <= fl_request )
			{ fl = fl_request; vz = 0.0f; }
		}
	// ici on doit tester s'il n'y a pas une requete de diversion, avant de tester cnt
	// possiblement la diversion va reinitialiser cnt et calculer une nouvelle route
	Beacon b;
	if	( diversion >= 0 )
		{				// diversion vers un autre waypoint
		curway = diversion;
		// si la diversion est dans le plan, iplan va permttre la continuation de ce plan
		// sinon iplan = -2 va faire quitter le plan apres cette diversion
		iplan = find_in_plan( curway );
		do	{
			if	( get_beacon( &b, curway ) == 0 )
				{
				if	( routetoXY( b.x, b.y ) )
					{ diversion = -1; return; }	// ok, on fait la directe dans le plan ou pas
				else 	{
					if	( iplan < 0 )
						{ adrift(); return; }	// c'etait la derniere chance
					iplan++;
					if	( iplan >= int(qplan) )			// on a atteint le bout du plan
						{ iplan = -2; adrift(); return; }	// le plan est fini
					curway = plan[iplan];		// on skippe sur la suite du plan
					}
				}
			else	adrift();
			} while (1);
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
			do	{
				iplan++;
				if	( iplan >= int(qplan) )			// on a atteint le bout du plan
					{ iplan = -2; adrift(); return; }	// le plan est fini
				// ici on sait ou aller
				curway = plan[iplan];
				if	( get_beacon( &b, curway ) == 0 )
					{
					if	( routetoXY( b.x, b.y ) )
						return;		// route ok, on sort de la boucle do
					}			// sinon "trop pres", on boucle pour chercher le waypoint suivant
				else	adrift();
				} while (1);
			} break;
		case 1: {	// fin premier virage d'une "route to XY"
			if	( get_beacon( &b, curway ) == 0 )
				{ gotoXY( b.x, b.y ); segtype = 0; }
			else	adrift();
			} break;
		case 3: {	// fin droite initiale (rare) d'une "route to XY"
			if	( get_beacon( &b, curway ) == 0 )
				routetoXY( b.x, b.y );
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
// au cap courant (d > 0.0)
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
	float dc = limit_cap( qfp_fsub( cap2, cap ) );
	if	( jfp_fsgn( dc ) )
		w = -w;
	cnt = (int)qfp_fadd( 0.5, qfp_fdiv( dc, w ) );
	}
// route depuis le point courant et le cap courant: virage puis segment droit, sauf si trop pres,
// alors si OPT_SKIP_TOO_CLOSE, abandon avec return 0
// ou sinon segment d'eloignement (segtype = 3) qui sera suivi d'un nouvel appel a routetoXY()
// return 1 si ok, 0 si skipped
int Apilot::routetoXY( float xb, float yb ) {
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
		}
	else	{	// C a gauche
		xc = qfp_fsub( x, qfp_fmul( r3, qfp_fsin( cap ) ) );
		yc = qfp_fadd( y, qfp_fmul( r3, qfp_fcos( cap ) ) );
		}
	//CDC_printf( "C %.5f %.5f\n", xc, yc );
// calculer distance de C a B (B = destination finale)
	float dx, dy;
	dx = qfp_fsub( xb, xc ); dy = qfp_fsub( yb, yc );
	float modcb = qfp_fsqrt( qfp_fadd( qfp_fmul( dx, dx ), qfp_fmul( dy, dy ) ) );
	//CDC_printf( "|CB| %.5f\n", modcb );
// mais est-ce possible ?
	if	( jfp_fsgn( qfp_fsub( modcb, r3 ) ) )	// si D est dans le cercle, on ne sait pas faire
		{
		CDC_printf("too close, too close\n");
		#ifdef OPT_SKIP_TOO_CLOSE
		return 0;
		#else
		// option s'eloigner en ligne droite car le point vise est trop proche
		gotoD( qfp_fmul( 2.0f, r3 ) );	// avec 2r on est sur (mais c'est trop dans la plupart des cas)
		segtype = 3;
		return 1;
		#endif
		}
// calculer premier virage
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
	CDC_printf( "== cap exact %.5f\n", qfp_fmul( ToDegrees, cape) );
// executer le virage (sens impose par signe de dcapro, possiblement > 180 deg)
	float dc = qfp_fsub( cape, cap );
	CDC_printf( "== dc        %.5f\n", qfp_fmul( ToDegrees, dc ) );
	dc = limit_cap( dc );
	CDC_printf( "== dc limitd %.5f\n", qfp_fmul( ToDegrees, dc ) );
	if	( jfp_fsgn(dcapro) )
		{
		w = -w3;
		if	( !jfp_fsgn( dc ) )
			dc = qfp_fsub( dc, PIx2 );
		}
	else	{
		w = w3;
		if	( jfp_fsgn( dc ) )
			dc = qfp_fadd( dc, PIx2 );
		}
	CDC_printf( "== dc fixed  %.5f\n", qfp_fmul( ToDegrees, dc ) );
	cnt = abs( (int)qfp_fadd( 0.5, qfp_fdiv( dc, w ) ) );
	segtype = 1;
	return 1;
	}

// calcul CRC32 AIXM
unsigned int crc_aixm( const unsigned char *buf, unsigned int len )
{
unsigned int crc = 0, i, msbin, msbreg, bit;
unsigned char lebyte;
do	{
	lebyte = *(buf++);
	for	( i = 0; i < 8; i++ )
		{
		msbin  = lebyte >> 7;
		msbreg = crc >> 31;
		bit = ( msbin ^ msbreg ) & 1;
		crc <<= 1;
		if	( bit )
			crc ^= 0x814141ABL;
		lebyte <<= 1;
        	}
	} while (--len);
return crc;
}

// verification de CRC32 dans packet p (le crc est a p + (p[0]-3))
// retour 1 si ok
int Apilot::CRC32ok( unsigned char * p )
{
unsigned int len = p[0];
unsigned int local_crc = crc_aixm( p + 1, len - 4 );
unsigned int rx_crc = from_32le( p + (len-3) );
if	( local_crc != rx_crc )
	{
	CDC_printf("BAD CRC %08x vs %08x\n", local_crc, rx_crc );
	return 0;
	}
CDC_printf("GOOD CRC %08x\n", rx_crc );
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
				{ queue_unable( BADWAY, p + (p[0]-3) ); return; }
			lepilot.diversion = (int)wpt;
			queue_wilco( p + (p[0]-3) );
			}
		break;
	case TURN:   if ( ( p[0] == 8 ) && ( CRC32ok(p) ) )
			{
			int newhead = from_16le( p+3 );
			if	( ( newhead < -180 ) || ( newhead > 360 ) )
				{ queue_unable( BADHDG, p + (p[0]-3) ); return; }
			lepilot.diversion = -3;
			lepilot.cap_diversion = lepilot.head2cap(float(newhead));
			//CDC_printf("cap_d = %d = %.3f\n", newhead, lepilot.cap_diversion );
			queue_wilco( p + (p[0]-3) );
			}
		break;
	case NEWFL:  if ( ( p[0] == 8 ) && ( CRC32ok(p) ) )
			{
			unsigned int newfl = from_16le( p+3 );
			if	( ( newfl < FLMIN ) || ( newfl > FLMAX ) )
				{ queue_unable( BADFL, p + (p[0]-3) ); return; }
			fl_request = float(newfl);
			if	( fl_request > fl )
				vz = vzup;
			else 	vz = vzdown;
			queue_wilco( p + (p[0]-3) );
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
to_16le( ubuf+3, short(qfp_fmul( x, 100.0f )) );
to_16le( ubuf+5, short(qfp_fmul( y, 100.0f )) );
to_16le( ubuf+7, short(qfp_fmul( vx, 36000.0f )) );	// convert Nm/s to knots*10
to_16le( ubuf+9, short(qfp_fmul( vy, 36000.0f )) );
to_16le( ubuf+11, short(fl) );
to_16le( ubuf+13, short(t) );
return CC.tx_if_can( ubuf );
}

// mise en queue d'un WILCO (revoie les 4 bytes du crc)
int Apilot::queue_wilco( unsigned char * crcbuf )
{
unsigned char ubuf[8];
ubuf[0] = 6;
ubuf[1] = FLIGHT | 0x80;
ubuf[2] = opcode_t(WILCO);
ubuf[3] = crcbuf[0];
ubuf[4] = crcbuf[1];
ubuf[5] = crcbuf[2];
ubuf[6] = crcbuf[3];
int retval = CC.tx_if_can( ubuf );
if	( retval )
	CDC_printf("WILCO failed %d\n", retval );
else	CDC_printf("WILCO\n");
return retval;
}

// mise en queue d'un UNABLE (revoie le byte d'err code suivi des 4 bytes de CRC)
int Apilot::queue_unable( err_t err, unsigned char * crcbuf )
{
unsigned char ubuf[8];
ubuf[0] = 7;
ubuf[1] = FLIGHT | 0x80;
ubuf[2] = opcode_t(UNABLE);
ubuf[3] = err;
ubuf[4] = crcbuf[0];
ubuf[5] = crcbuf[1];
ubuf[6] = crcbuf[2];
ubuf[7] = crcbuf[3];
int retval = CC.tx_if_can( ubuf );
if	( retval )
	CDC_printf("UNABLE failed %d\n", retval );
else	CDC_printf("UNABLE %02x\n", err );
return retval;
}

