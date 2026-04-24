# Topic de statut du face follower
# Publié en continu pour que le SBE puisse monitorer l'état
# header.frame_id = "base_link" (repère de la base du robot)

std_msgs/Header header

# États de tracking possibles
uint8 IDLE = 0           # Pas de tracking actif
uint8 SEARCHING = 1      # Recherche d'un visage
uint8 TRACKING = 2       # Suivi actif d'un visage
uint8 BLOCKED = 3        # Bloqué (limite articulaire, etc.)

uint8 tracking_state     # État actuel du tracking

# Capacités activées (contrôlées par SetFollowerStatus service)
bool head_movement_enabled
bool base_rotation_enabled

# Raison de blocage (vide si tracking_state != BLOCKED)
string blocking_reason   # Ex: "head_near_border", "manual_disable"

# Position 3D du visage détecté dans le repère base_link (base du robot)
# Ces coordonnées sont calculées en tenant compte de l'orientation de la tête
# Repère base_link : X = avant, Y = gauche, Z = haut
float32 face_distance    # Distance au visage (mètres)
float32 face_x           # Coordonnée X dans base_link (mètres, avant du robot)
float32 face_y           # Coordonnée Y dans base_link (mètres, gauche du robot)
float32 face_z           # Coordonnée Z dans base_link (mètres, haut du robot)

# Position actuelle de la tête (retour de /joint_states)
float32 head_pan         # Position pan actuelle (radians)
float32 head_tilt        # Position tilt actuelle (radians)

# Position du visage dans l'image (coordonnées centrées, pour debug)
float32 face_u           # Position horizontale centrée (pixels)
float32 face_v           # Position verticale centrée (pixels)

# Nombre de visages détectés
uint8 faces_detected     # Nombre total de visages dans l'image


