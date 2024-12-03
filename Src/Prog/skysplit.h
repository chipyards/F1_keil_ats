#define PI ((float)3.14159265358)
#define ToRadians ((float)(PI/180.0))
#define ToDegrees ((float)(180.0/PI))
#define QBEACON 11
#define QPLAN 16
extern volatile unsigned int cnt100Hz;
extern volatile unsigned int cnt1Hz;

class Beacon {
public:
float x;
float y;
};

extern const float rom_beacons[];

class Apilot {
public:
// etat du mouvement
float x;
float y;
float cap;		// radian, repere trigo
float w;		// taux de virage en rad/s, signed
float vx;		// derive de v et cap
float vy;		// derive de v et cap
// parametres du mouvement
float v;	// vitesse en Nm/step 0.1 <==> 360 knots
float w3;	// taux de virage t.q. 3 deg/s
float r3;	// rayon de virage derive de v et w3 (pour 3 deg/s : 1.9 NM @ 360 knots)
float cap_diversion;	// cap demande en cas de virage de diversion
// donnees de plan
const Beacon * beacons;	// base de la table des balises
int plan[QPLAN];
int qplan;	// nombre de waypoints dans le plan
int iplan;	// index dans le plan

// FSM
int cnt;		// steps restant avant le prochain segment (-1 si infini)
int segtype;		// 0 = tout segment atteignant un waypoint
			// 1 = premier virage d'une "route to XY"
			// 0 = droite finale d'une "route to XY"
			// 3 = droite initiale (rare) d'une "route to XY"
			// 4 = virage de diversion
int target_waypoint;	// >= 0 : indice du waypoint vers lequel on va
			// -2 = pas de waypoint, on continue tout droit
// flag temporaire
int diversion;		// -1 : pas de diversion en cours
			// >= 0 : indice du nouveau waypoint pour lequel on doit calculer une trajectoire
			// -2 = pas de waypoint, on continue tout droit
			// -3 = deroutement demandé : changement de cap puis tout droit

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
	cnt = 0;
	target_waypoint = -2;
	diversion = -1;
	cap_diversion = 0.0;
	beacons = (Beacon *)rom_beacons;
	adrift();
	int i = 0;
	plan[i++] = 1;	plan[i++] = 2;	plan[i++] = 3;	plan[i++] = 4;
	plan[i++] = 5;	plan[i++] = 6;	plan[i++] = 7;	plan[i++] = 8;	plan[i++] = 9;	plan[i++] = 10;	plan[i++] = 0;
	qplan = i;
	iplan = 0;
	};

// // accesseurs
const Beacon * get_beacon( int i ) {
	if	 ( i < QBEACON )
		return &(beacons[i]);
	else	return 0;
	};

// lire la suite du plan de vol
int get_next_waypoint() {
	if	( ( iplan > ( qplan - 1 ) ) || ( iplan < 0 ) )
		return -2;
	else	return plan[iplan++];
	};

// // methodes de calcul
// ramener cap dans ] -PI/2, +PI/2 ]
float limit_cap( float c );
// conversions
//	head = cap en degres dans [ 0, 360 [
//	cap en radian dans ] -PI/2, +PI/2 ]
float head2cap( float h );
float cap2head( float c );
// emet un report
void dump_loc();
// calcul le cap du premier virage
float angletoXY( float xb, float yb );

// // methodes configurant la FSM pour les prochains steps
// fuir tout droit en attendant un ordre
void adrift();
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

// // step de la FSM (une seconde pour le moment)
void step();

}; // class Apilot

extern Apilot lepilot;

void test_a();
void test_b();


