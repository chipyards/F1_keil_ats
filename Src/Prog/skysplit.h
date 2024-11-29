#define PI ((float)3.14159265358)
#define ToRadians ((float)(PI/180.0))
#define ToDegrees ((float)(180.0/PI))

class Apilot {
public:
float x;
float y;
float v;	// vitesse en Nm/s 0.1 <==> 360 knots
float vx;
float vy;
float cap;	// radian, repere trigo
float w;	// taux de virage en rad/s, signed
float w3;	// 3 deg/s
float r3;	// rayon de virage pour 3 deg/s (1.9 NM @ 360 knots)

Apilot() {
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
	};

// // methodes de calcul
// ramener cap dans ] -PI/2, +PI/2 ]
float limit_cap( float c );
// notre convention:
//	head = cap en degres dans [ 0, 360 [
//	cap en radian dans ] -PI/2, +PI/2 ]
float head2cap( float h );
float cap2head( float c );
float angletoXY( float xb, float yb );

void demo();
};


void test_a();
void test_b();


