// Parseur d'arguments de ligne de commande rustique

// accepte dans n'importe quel ordre :
//	- des options sans valeur, de la forme -X ou -Xblabla
//	- des options avec valeur, de la forme -Xval ou -X=val ou -X val
//	  la clef de l'option X etant une lettre [A-Za-z]
//	- une chaine nue t.q. file path
// les clefs qui attendent une valeur doivent etre listees dans value_expecting_keys

// l'application recupere la valeur pour chaque clef avec la methode get
// la clef speciale '@' sert a recuperer la chaine nue
// la valeur est:
//	- NULL si la clef n'est pas parmi les arguments
//	- 1 si la clef est presente mais n'attend pas de valeur
//	- un pointeur sur une chaine si la clef est presente et attend une valeur

// detection d'erreurs
//	- le parseur ne detecte pas d'erreur
//	- si une valeur manque, get() rend l'argument suivant comme valeur
//	  (donc il est perdu), ou une chaine vide si c'est le dernier argument
//	- si une clef est dupliquee, get() rend la derniere valeur
//	- si la chaine nue est dupliquee, get() rend la derniere
// l'application doit verifier que chaque valeur est conforme a ses attentes

class cli_parse {

private:

const char * vals[64];	// valeur associee a chaque clef
char val_flags[64];	// indique pour chaque clef si elle attend une valeur

// comprimer lettres ascii sur [0:63]
// N.B. les car de ponctuation sont replies, par exemple '-' <==> 'm' 
int c2i( char c ) { return ( c - 0x40 ) & 0x3F; };

public:

// constructeur, effectue le parsage et memorise les resultats
cli_parse( int argc, const char ** argv, const char * value_expecting_keys )
{
for	( int i = 0; i < 64; ++i )
	{ vals[i] = NULL; val_flags[i] = 0; }
char key;
const char * val;
while	( ( key = *(value_expecting_keys++) ) )
	val_flags[c2i(key)] = 1;
for	( int iopt = 1; iopt < argc; ++iopt )
	{
	if	( ( argv[iopt][0] == '-' ) && ( argv[iopt][1] > ' ' ) )
		{
		key = argv[iopt][1];
		if	( val_flags[c2i(key)] )
			{
			val = argv[iopt]+2;		// valeur collee a la lettre clef
			if	( *val == '=' )
				val += 1;		// valeur apres signe '='
			else if	( *val == 0 )	
				{			// valeur separee
				if	( (++iopt) < argc )
					val = argv[iopt];
				}
			}
		else	val = (const char *)1;		// indique juste presence de la clef
		vals[c2i(key)] = val;
		}
	else	vals[0] = argv[iopt];			// clef '@' <==> chaine nue (file name) 
	}
};
// methode
const char * get( char key ) {
	return vals[c2i(key)];
	};
};

