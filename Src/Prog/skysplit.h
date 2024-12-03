#define PI ((float)3.14159265358)
#define ToRadians ((float)(PI/180.0))
#define ToDegrees ((float)(180.0/PI))

extern volatile unsigned int cnt100Hz;
extern volatile unsigned int cnt1Hz;

class Apilot {
public:
float x;
float y;
float v;	// vitesse en Nm/step 0.1 <==> 360 knots
float vx;
float vy;
float cap;	// radian, repere trigo
float w;	// taux de virage en rad/s, signed
float w3;	// 3 deg/s
float r3;	// rayon de virage pour 3 deg/s (1.9 NM @ 360 knots)
unsigned int ds;	// duree du step en sys ticks (0.01s)
unsigned int next_step; // timestamp du next step en ticks abs


Apilot() {	// constructeur
	init();
	};

void init() {
	x = 0.0f;
        y = 0.0f;
        v = 0.1f;		// vitesse en Nm/s 0.1 <==> 360 knots
        vx = v;
        vy = 0.0;
        cap = 0.0;		// radian, repere trigo
        w = 0.0;			// taux de virage en rad/s, signed
        w3 = qfp_fmul( ToRadians, 3 );	// 3 deg/s
        r3 = qfp_fdiv( v, w3 );		// rayon de virage pour 3 deg/s (1.9 NM @ 360 knots)
        ds = 25;
        next_step = 0;
	};

// // methodes de calcul
// ramener cap dans ] -PI/2, +PI/2 ]
float limit_cap( float c );
// notre convention:
//	head = cap en degres dans [ 0, 360 [
//	cap en radian dans ] -PI/2, +PI/2 ]
float head2cap( float h );
float cap2head( float c );
void dump_loc();
float angletoXY( float xb, float yb );
// // methodes de simulation, iterent step()
// step d'une seconde (pour le moment)
void step();
// simple segment de droite de longueur d depuis le point courant x, y
// au cap courant
void gotoD( float d );
// simple segment de droite depuis le point courant x, y (recalcule le cap)
void gotoXY( float xd, float yd );
// arc de cercle depuis le point courant x, y et le cap courant
// sens automatique (virage < 180 deg)
void turnTo( float cap2 );
// arc de cercle depuis le point courant x, y et le cap courant
// en imposant le taux (w) et le sens de rotation (signe de w)
void turnTo( float cap2, float w );
// route depuis le point courant et le cap courant: virage puis segment, ou si trop pres,
// segment puis virage puis segment
void routetoXY( float xb, float yb );

void demo();
}; // class Apilot

extern Apilot lepilot;

void test_a();
void test_b();


