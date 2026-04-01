# Topic de statut du face follower
# Publié en continu pour que le SBE puisse monitorer l'état

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

# Position et distance du visage détecté
float32 face_distance    # Distance au visage (mètres)
float32 face_x           # Coordonnée X dans le repère robot (mètres, avant)
float32 face_y           # Coordonnée Y dans le repère robot (mètres, gauche)
float32 face_z           # Coordonnée Z dans le repère robot (mètres, haut)

# Position actuelle de la tête (retour de /joint_states)
float32 head_pan         # Position pan actuelle (radians)
float32 head_tilt        # Position tilt actuelle (radians)

# Position du visage dans l'image (coordonnées centrées, pour debug)
float32 face_u           # Position horizontale centrée (pixels)
float32 face_v           # Position verticale centrée (pixels)

# Nombre de visages détectés
uint8 faces_detected     # Nombre total de visages dans l'image


