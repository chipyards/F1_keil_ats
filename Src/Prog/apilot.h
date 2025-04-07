#define PI ((float)3.14159265358)
#define ToRadians ((float)(PI/180.0))
#define ToDegrees ((float)(180.0/PI))
#define QBEACON 11
#define QPLAN 56

// opcodes
enum opcode_t {
	NEWFP=0x70, DIRECT=0x4A, TURN=0x5E, NEWFL=0x14,
	WILCO=0, UNABLE=1,
	REQFP=0x71, REQWCO=0x75, REQALT=0x15, REQRAT=0x79,
	REPFP=0x72, REPWCO=0x76, REPALT=0x16, REPRAT=0x7A,
	VAAR=0x42, PAUSE=0x81, RESUME=0x82, RATECK=0x83, SRESET=0x84
	};
// errors
enum err_t { BADWAY=0x70, BADHDG=0x5E, BADFL=0x14 };

// defaults
#define FLMIN (100)
#define FLMAX (380)

class Beacon {
public:
float x;
float y;
};

extern const float rom_beacons[];

class Apilot {
public:
// config simu
unsigned int sim_speed;	// acceleration virtuelle du temps avion
// etat du mouvement
unsigned int t;		// timestamp
float fl;		// flight level
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
unsigned char plan[QPLAN];
unsigned int qplan;	// nombre de waypoints dans le plan

// autopilot FSM
int cnt;		// steps restant avant le prochain segment (-1 si infini)

int segtype;		// 0 = tout segment atteignant un waypoint
			// 1 = premier virage d'une "route to XY"
			// 0 = droite finale d'une "route to XY"
			// 3 = droite initiale (rare) d'une "route to XY"
			// 4 = virage de diversion
			// 5 = drift

int iplan;		// index dans le plan du waypoint courant (vers lequel on va)
			// -2 si on n'est plus dans le plan

int curway;		// way point courant, egal a plan[iplan] sauf si on n'est pas dans le plan

int diversion;		// -1 : pas de diversion en cours (les autres valeurs sont temporaires)
			// >= 0 : nouveau waypoint pour lequel on doit calculer une trajectoire
			// -2 = pas de waypoint, on continue tout droit (inutilise)
			// -3 = deroutement demandé : changement de cap puis tout droit
// divers
unsigned int rxCRC;

Apilot() {	// constructeur
	init();
	};

void load_plan();
void init();

// // accesseurs
const Beacon * get_beacon( int i ) {
	if	 ( i < QBEACON )
		return &(beacons[i]);
	else	return 0;
	};

// chercher un waypoint dans le plan (-2 si pas trouve)
int find_in_plan( unsigned int wpt ) {
	unsigned int i;
	for	( i = 0; i < qplan; i++ )
		{
		if	( wpt == plan[i] )
			return (int)i;
		}
	return -2;
	}

// // methodes de calcul
// ramener cap dans ] -PI/2, +PI/2 ]
float limit_cap( float c );
// conversions
//	head = cap en degres dans [ 0, 360 [
//	cap en radian dans ] -PI/2, +PI/2 ]
float head2cap( float h );
float cap2head( float c );
// emet un report vers CDC
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

// binary coding methods
void to_s16le( unsigned char * buf, float f ) {
	short s = (short)f;
	buf[0] = s;
	buf[1] = s >> 8;
	}
void to_u16le( unsigned char * buf, float f ) {
	unsigned short s = (unsigned short)f;
	buf[0] = s;
	buf[1] = s >> 8;
	}
void to_u16le( unsigned char * buf, unsigned int u ) {
	buf[0] = u;
	buf[1] = u >> 8;
	}
unsigned int from_u16le( unsigned char * buf ) {
	return buf[0] | ( buf[1] << 8 );
	}

// verification de CRC32 dans packet p
int CRC32ok( unsigned char * p );

// interpreteur de commandes (paquet radio, 1er byte is LEN, ADR et CRC deja verifies)
void cmd_handler( unsigned char * p );

// navigation automatic report, including navigation steps
int AAR_tx();

// mise en queue d'un WILCO (revoie les 4 bytes du crc)
void queue_wilco( unsigned char * crcbuf );

// mise en queue d'un UNABLE (revoie le byte derrcode suivi des 4 bytes de CRC)
void queue_unable( err_t err, unsigned char * crcbuf );

}; // class Apilot

extern Apilot lepilot;

void test_a();
void test_b();


