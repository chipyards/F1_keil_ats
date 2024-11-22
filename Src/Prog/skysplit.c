// simulation du pilote automatique: calcul de trajectoire
#include "qfplib-m3.h"
#include "skysplit.h"
#include "CDC.h"

void test_unitaire() {
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

/*
// route depuis le point courant et le cap courant: virage puis segment
float angletoXY( autopilot AP, float xb, float yb ) {
// decider de quel cote tourner : cap approximatif
	//float capro = Math.atan2( yb - y, xb - x );
	float capro = qfp_fatan2( qfp_fsub( yb, AP.y ), qfp_fsub( xb, AP.x ) );
	//float dcapro = limit_cap( capro - cap );
	float dcapro = limit_cap( qfp_fsub( capro, cap ) );
	//System.out.println( "capro=" + cap2head( capro ) + ", dcapro=" + cap2head( dcapro ) );
	CDCprintf( "capro=%.5f, dcapro=%.5f\n", cap2head( capro ), qfp_fmul( ToDegrees, dcapro ) );
	// ici le signe de dcapro indique le sens du virage
	// chercher le centre C de l'arc de cercle
	float xc, yc;
	if	( dcapro > 0 )
		{	// C a gauche
	//	xc = x - r3 * Math.sin(cap);
	//	yc = y + r3 * Math.cos(cap);
		w = w3;
		}
	else	{	// C a droite
	//	xc = x + r3 * Math.sin(cap);
	//	yc = y - r3 * Math.cos(cap);
		w = -w3;
		}
	System.out.println( "C " + xc + " " + yc );
	// distance de C a B (B = destination finale)
	float dx, dy;
	//dx = xb - xc; dy = yb - yc;
	//float modcb = Math.sqrt( dx * dx + dy * dy );
	System.out.println( "|CB| " + modcb );
	//if	( modcb < r3 )	// si D est dans le cercle, on ne sait pas faire
		{ System.out.println("too close, giving up"); return; }
	// angle CBD (D = fin virage), non signé et aigu
	//float cbd = Math.asin( r3 / modcb );
	System.out.println( "CBD " + Math.toDegrees(cbd) );
	// argument de CB
	//float argcb = Math.atan2( yb - yc, xb - xc );
	System.out.println( "arg CB " + cap2head(argcb) );
	// cap exact
	//float cape;
	//if	( dcapro > 0 )
	//	cape = argcb + cbd;
	//else	cape = argcb - cbd;
	System.out.println( "cap exact " + cap2head(cape) );
	return cape;
	}
*/
