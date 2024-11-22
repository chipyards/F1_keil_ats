#define PI ((float)3.14159265358)
#define ToRadians ((float)(PI/180.0))
#define ToDegrees ((float)(180.0/PI))

typedef struct {
	double x;
	double y;
	double v;	// vitesse en Nm/s 0.1 <==> 360 knots
	double vx;
	double vy;
	double cap;	// radian, repere trigo
	double w;	// taux de virage en rad/s, signed
	double w3;	// 3 deg/s
	double r3;	// rayon de virage pour 3 deg/s (1.9 NM @ 360 knots)
} autopilot;


void test_unitaire();

// ramener cap dans ] -PI/2, +PI/2 ]
float limit_cap( float c );

// notre convention:
//	head = cap en degres dans [ 0, 360 [
//	cap en radian dans ] -PI/2, +PI/2 ]
float head2cap( float h );
float cap2head( float c );

void autopilot_init( autopilot *AP );
float angletoXY( autopilot *AP, float xb, float yb );
