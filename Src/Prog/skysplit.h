#define PI ((float)3.14159265358)
#define ToRadians ((float)(PI/180.0))
#define ToDegrees ((float)(180.0/PI))

typedef struct {
	float x;
	float y;
	float v;	// vitesse en Nm/s 0.1 <==> 360 knots
	float vx;
	float vy;
	float cap;	// radian, repere trigo
	float w;	// taux de virage en rad/s, signed
	float w3;	// 3 deg/s
	float r3;	// rayon de virage pour 3 deg/s (1.9 NM @ 360 knots)
} autopilot;


void test_a();
void test_b();

// ramener cap dans ] -PI/2, +PI/2 ]
float limit_cap( float c );

// notre convention:
//	head = cap en degres dans [ 0, 360 [
//	cap en radian dans ] -PI/2, +PI/2 ]
float head2cap( float h );
float cap2head( float c );

void autopilot_init( autopilot *AP );
float angletoXY( autopilot *AP, float xb, float yb );
