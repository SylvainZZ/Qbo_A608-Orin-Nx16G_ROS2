# qbo_home_assistant — Documentation

## Principe de fonctionnement

### Architecture générale

```text
┌─────────────────────────────────────────────────────────────────┐
│  Environnement ROS2 (Qbo)                                       │
│                                                                 │
│   ┌──────────────────────┐    topics       ┌──────────────────┐ │
│   │  Nœuds Qbo           │◄──────────────  │  ha_bridge node  │ │
│   │  (NLP / AIML / TTS)  │                 │                  │ │
│   │                      │───services───►  │  ha_websocket.py │ │
│   └──────────────────────┘                 │  ha_client.py    │ │
│                                            └────────┬─────────┘ │
└─────────────────────────────────────────────────────┼───────────┘
                                                      │ WebSocket
                                                      │ ws://IP:8123
                                            ┌─────────▼─────────┐
                                            │   Home Assistant  │
                                            │   (KNX / capteurs │
                                            │    / volets…)     │
                                            └───────────────────┘
```

### Flux de données

#### États → topics (push temps réel)

Home Assistant pousse chaque changement d'état via son API WebSocket (événement
`state_changed`). Le nœud `ha_bridge` reçoit ces événements dans sa boucle
asyncio et les publie immédiatement sur le topic `~/entity_state`. Les nœuds
Qbo (NLP, décision) s'abonnent à ce topic et maintiennent ainsi un contexte
courant de l'ensemble du bâtiment sans jamais interroger HA directement.

#### Commandes → services (request/response)

Pour agir sur le bâtiment, les nœuds Qbo appellent un service ROS2. Le handler
bloque le temps de la requête WebSocket vers HA (≤ 15 s timeout), puis retourne
le résultat. Les trois services disponibles couvrent tous les cas d'usage :

| Service ROS2              | Usage                                      |
|---------------------------|--------------------------------------------|
| `~/call_service`          | Envoyer n'importe quelle commande à HA     |
| `~/get_entity_state`      | Lire l'état courant d'une entité           |
| `~/get_all_states`        | Lire toutes les entités (filtre optionnel) |

#### Reconnexion automatique

Si la connexion WebSocket est perdue (redémarrage HA, coupure réseau), le nœud
tente de se reconnecter toutes les `reconnect_delay_s` secondes. L'état de la
connexion est publié en continu sur le topic latché `~/connection` : un nœud
Qbo peut surveiller ce topic pour adapter son comportement verbal
(« Je ne peux pas accéder à la maison pour l'instant »).

### Modèle de concurrence

L'API WebSocket de HA est asynchrone (asyncio). ROS2 Python est synchrone
(callbacks dans l'exécuteur rclpy). Le pont est réalisé ainsi :

- Un thread dédié (`ha_asyncio`) fait tourner `asyncio.run_forever()`.
- Les handlers de service utilisent `asyncio.run_coroutine_threadsafe(...).result()`
  pour soumettre une coroutine depuis le thread ROS2 et attendre le résultat.
- Les événements HA sont publiés directement depuis le thread asyncio ; les
  publishers rclpy sont thread-safe.
- Un `_recv_loop` interne à `HAWebSocket` multiplex les frames par ID de message,
  ce qui permet d'avoir une souscription `state_changed` persistante **et** des
  appels request/response concurrents sur la même connexion.

---

## Mise en œuvre

### Prérequis

- ROS2 Humble (ou supérieur) installé sur la machine Qbo
- Python ≥ 3.10
- Package `websockets` : `pip install websockets`
- Token d'accès longue durée Home Assistant (voir ci-dessous)

### Obtenir le token Home Assistant

1. Se connecter à l'interface web HA (`http://IP:8123`)
2. Cliquer sur le profil utilisateur (icône en bas à gauche)
3. Descendre jusqu'à **Jetons d'accès longue durée** → **Créer un jeton**
4. Nommer le jeton `qbo` et copier la valeur (visible une seule fois)

### Structure du workspace ROS2

```text
ros2_ws/
└── src/
    ├── qbo_ha_interfaces/     ← interfaces (msgs / srvs) — build ament_cmake
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── msg/
    │   │   ├── EntityState.msg
    │   │   └── HaConnectionStatus.msg
    │   └── srv/
    │       ├── CallHaService.srv
    │       ├── GetEntityState.srv
    │       └── GetAllStates.srv
    └── qbo_home_assistant/    ← nœud Python — build ament_python
        ├── package.xml
        ├── setup.py
        ├── config/
        │   └── ha_bridge_params.yaml
        ├── launch/
        │   └── ha_bridge.launch.py
        └── qbo_home_assistant/
            ├── ha_websocket.py     # client WS bas niveau
            ├── ha_client.py        # helpers haut niveau
            ├── ha_bridge_node.py   # nœud ROS2
            └── main.py
```

### Compilation

```bash
cd ~/ros2_ws

# Installer les dépendances système
rosdep install --from-paths src --ignore-src -r -y

# Compiler dans l'ordre (interfaces d'abord)
colcon build --packages-select qbo_ha_interfaces
colcon build --packages-select qbo_home_assistant

# Sourcer l'environnement
source install/setup.bash
```

### Configuration

Éditer `config/ha_bridge_params.yaml` avant le lancement :

```yaml
ha_bridge:
  ros__parameters:
    ha_ws_url: "ws://192.168.1.100:8123/api/websocket"  # IP de votre HA
    ha_token: "eyJhbGciOiJIUzI1NiIs..."                 # token copié ci-dessus
    reconnect_delay_s: 5.0
    all_states_period_s: 0.0   # > 0 pour activer un snapshot périodique complet
```

### Lancement

#### Via le fichier de paramètres

```bash
ros2 launch qbo_home_assistant ha_bridge.launch.py \
  ha_ws_url:=ws://192.168.1.100:8123/api/websocket \
  ha_token:=eyJhbGciOiJIUzI1NiIs...
```

#### Via le fichier YAML

```bash
ros2 run qbo_home_assistant ha_bridge \
  --ros-args --params-file src/qbo_home_assistant/config/ha_bridge_params.yaml
```

---

## Tests

### 1. Vérifier que le nœud tourne et qu'il est connecté

```bash
# Dans un terminal
ros2 topic echo /ha_bridge/connection
```

Sortie attendue (connexion établie) :

```yaml
stamp: {sec: 1753920000, nanosec: 0}
connected: true
ha_url: ws://192.168.1.100:8123/api/websocket
message: Connected
```

### 2. Surveiller les changements d'état en temps réel

```bash
ros2 topic echo /ha_bridge/entity_state
```

Appuyer sur un interrupteur KNX depuis le bâtiment ; la sortie doit afficher
immédiatement l'événement correspondant :

```yaml
stamp: {sec: 1753920145, nanosec: 320000000}
entity_id: light.chambre_1
domain: light
friendly_name: Chambre 1
state: 'on'
attributes: '{"friendly_name": "Chambre 1"}'
area_id: ''
---
stamp: {sec: 1753920147, nanosec: 80000000}
entity_id: sensor.temperature_exterieure
domain: sensor
friendly_name: Température extérieure
state: '18.4'
attributes: '{"unit_of_measurement": "°C", "device_class": "temperature",
              "state_class": "measurement"}'
area_id: ''
```

---

## Exemples de service — installation réelle

Les `entity_id` sont dérivés des noms définis dans `knx.yaml` par slugification
(minuscules, accents supprimés, espaces → `_`).

---

### Éclairage (`light`)

#### Allumer le Salon

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'light', service: 'turn_on', entity_id: 'light.salon', service_data: ''}"
```

```yaml
success: true
result: '{"id": 12, "type": "result", "success": true, "result": {"context": {"id": "abc123"}}}'
error_message: ''
```

#### Allumer le Salon avec dimmer à 40 %

(Le Salon dispose d'une adresse `brightness_address` — le dimmer est supporté.)

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'light', service: 'turn_on', entity_id: 'light.salon', \
    service_data: '{\"brightness_pct\": 40}'}"
```

```yaml
success: true
result: '{"id": 13, "type": "result", "success": true, "result": {"context": {"id": "def456"}}}'
error_message: ''
```

#### Éteindre la lumière du Bureau

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'light', service: 'turn_off', entity_id: 'light.bureau', service_data: ''}"
```

```yaml
success: true
result: '{"id": 14, "type": "result", "success": true, "result": {"context": {"id": "ghi789"}}}'
error_message: ''
```

#### Éteindre toutes les lumières

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'light', service: 'turn_off', entity_id: '', service_data: ''}"
```

> Sans `entity_id`, HA éteint toutes les entités du domaine `light`.
> Pour ne cibler qu'une pièce, fournir l'`area_id` dans `service_data` :
> `service_data: '{"area_id": "salon"}'`

#### Lister toutes les lumières et leur état

```bash
ros2 service call /ha_bridge/get_all_states qbo_ha_interfaces/srv/GetAllStates \
  "{domain_filter: 'light'}"
```

```yaml
success: true
states:
  - entity_id: light.atelier
    friendly_name: Atelier
    state: 'off'
    attributes: '{"friendly_name": "Atelier"}'
  - entity_id: light.bureau
    friendly_name: Bureau
    state: 'on'
    attributes: '{"friendly_name": "Bureau"}'
  - entity_id: light.salon
    friendly_name: Salon
    state: 'on'
    attributes: '{"brightness": 102, "friendly_name": "Salon"}'
  # … 23 autres lumières
error_message: ''
```

---

### Volets roulants (`cover`)

#### Fermer le volet du Salon

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'cover', service: 'close_cover', \
    entity_id: 'cover.volet_roulant_sw_salon', service_data: ''}"
```

```yaml
success: true
result: '{"id": 15, "type": "result", "success": true, "result": {"context": {"id": "jkl012"}}}'
error_message: ''
```

#### Ouvrir le volet du Bureau

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'cover', service: 'open_cover', \
    entity_id: 'cover.volet_roulant_sw_bureau', service_data: ''}"
```

```yaml
success: true
result: '{"id": 16, "type": "result", "success": true, "result": {"context": {"id": "mno345"}}}'
error_message: ''
```

#### Positionner le volet de Cuisine à 30 % (protection solaire)

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'cover', service: 'set_cover_position', \
    entity_id: 'cover.volet_roulant_se_cuisine', \
    service_data: '{\"position\": 30}'}"
```

```yaml
success: true
result: '{"id": 17, "type": "result", "success": true, "result": {"context": {"id": "pqr678"}}}'
error_message: ''
```

#### Arrêter le volet de la Salle à manger A

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'cover', service: 'stop_cover', \
    entity_id: 'cover.volet_roulant_a_nw_salle_a_manger', service_data: ''}"
```

#### Lister tous les volets et leurs positions

```bash
ros2 service call /ha_bridge/get_all_states qbo_ha_interfaces/srv/GetAllStates \
  "{domain_filter: 'cover'}"
```

```yaml
success: true
states:
  - entity_id: cover.volet_roulant_sw_salon
    friendly_name: Volet roulant_SW Salon
    state: closed
    attributes: '{"current_position": 0, "device_class": "shutter"}'
  - entity_id: cover.volet_roulant_se_cuisine
    friendly_name: Volet roulant_SE Cuisine
    state: open
    attributes: '{"current_position": 70, "device_class": "shutter"}'
  - entity_id: cover.volet_roulant_nw_chambre_2
    friendly_name: Volet roulant_NW Chambre 2
    state: open
    attributes: '{"current_position": 100, "device_class": "shutter"}'
  # … 9 autres volets
error_message: ''
```

---

### Commutateurs (`switch`)

#### Ouvrir le Portail

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'switch', service: 'turn_on', entity_id: 'switch.portail', service_data: ''}"
```

```yaml
success: true
result: '{"id": 18, "type": "result", "success": true, "result": {"context": {"id": "stu901"}}}'
error_message: ''
```

#### Ouvrir la Porte Garage

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'switch', service: 'turn_on', entity_id: 'switch.porte_garage', service_data: ''}"
```

#### Activer / désactiver le Cumulus

```bash
# Activer
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'switch', service: 'turn_on', entity_id: 'switch.cumulus', service_data: ''}"

# Désactiver
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'switch', service: 'turn_off', entity_id: 'switch.cumulus', service_data: ''}"
```

---

### Chauffage — mode fil pilote (`select`)

Modes disponibles : `Off` / `Confort` / `Eco` / `Hors gel`

#### Passer le Chauffage Bureau en mode Eco

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'select', service: 'select_option', \
    entity_id: 'select.chauffage_bureau', \
    service_data: '{\"option\": \"Eco\"}'}"
```

```yaml
success: true
result: '{"id": 19, "type": "result", "success": true, "result": {"context": {"id": "vwx234"}}}'
error_message: ''
```

#### Mettre Chambre 1 en Confort

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'select', service: 'select_option', \
    entity_id: 'select.chauffage_chambre_1', \
    service_data: '{\"option\": \"Confort\"}'}"
```

#### Hors gel pour toute la maison (Chambre 1, 2, Bureau, Atelier)

```bash
for entity in chauffage_chambre_1 chauffage_chambre_2 chauffage_bureau chauffage_atelier; do
  ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
    "{domain: 'select', service: 'select_option', entity_id: 'select.${entity}', service_data: '{\"option\": \"Hors gel\"}'}" 
done
```

> La payload YAML est sur une seule ligne pour que `${entity}` s'expande
> correctement dans la chaîne double-quotée du shell.

---

### Chauffage — clim / plancher chauffant (`climate`)

#### Régler la consigne de Chauffage Zone 3 à 21 °C

```bash
ros2 service call /ha_bridge/call_service qbo_ha_interfaces/srv/CallHaService \
  "{domain: 'climate', service: 'set_temperature', \
    entity_id: 'climate.chauffage_zone_3', \
    service_data: '{\"temperature\": 21.0}'}"
```

```yaml
success: true
result: '{"id": 20, "type": "result", "success": true, "result": {"context": {"id": "yza567"}}}'
error_message: ''
```

#### Lire la consigne et la température mesurée de Zone 3

```bash
ros2 service call /ha_bridge/get_entity_state qbo_ha_interfaces/srv/GetEntityState \
  "{entity_id: 'climate.chauffage_zone_3'}"
```

```yaml
success: true
entity_state:
  entity_id: climate.chauffage_zone_3
  domain: climate
  friendly_name: Chauffage Zone 3
  state: heat
  attributes: '{"temperature": 21.0,
                "current_temperature": 19.5,
                "hvac_modes": ["off", "heat", "cool", "heat_cool"],
                "fan_mode": "auto",
                "friendly_name": "Chauffage Zone 3"}'
  area_id: ''
error_message: ''
```

---

### Capteurs — météo et environnement (`sensor`)

#### Lire la température extérieure

```bash
ros2 service call /ha_bridge/get_entity_state qbo_ha_interfaces/srv/GetEntityState \
  "{entity_id: 'sensor.temperature_exterieure'}"
```

```yaml
success: true
entity_state:
  entity_id: sensor.temperature_exterieure
  domain: sensor
  friendly_name: Température extérieure
  state: '18.4'
  attributes: '{"unit_of_measurement": "°C", "device_class": "temperature",
                "state_class": "measurement"}'
  area_id: ''
error_message: ''
```

#### Lire la luminosité extérieure (pilotage volets)

```bash
ros2 service call /ha_bridge/get_entity_state qbo_ha_interfaces/srv/GetEntityState \
  "{entity_id: 'sensor.luminosite_exterieure'}"
```

```yaml
success: true
entity_state:
  entity_id: sensor.luminosite_exterieure
  state: '34500'
  attributes: '{"unit_of_measurement": "lx", "device_class": "illuminance",
                "state_class": "measurement"}'
```

#### Lire la température du Salon

```bash
ros2 service call /ha_bridge/get_entity_state qbo_ha_interfaces/srv/GetEntityState \
  "{entity_id: 'sensor.thermometre_salon_1_1_82'}"
```

```yaml
success: true
entity_state:
  entity_id: sensor.thermometre_salon_1_1_82
  state: '20.1'
  attributes: '{"unit_of_measurement": "°C", "device_class": "temperature"}'
```

#### Lire tous les capteurs (température, énergie, eau…)

```bash
ros2 service call /ha_bridge/get_all_states qbo_ha_interfaces/srv/GetAllStates \
  "{domain_filter: 'sensor'}"
```

---

### Capteurs binaires — alarmes et sécurité (`binary_sensor`)

#### Vérifier l'état de l'alarme intrusion

```bash
ros2 service call /ha_bridge/get_entity_state qbo_ha_interfaces/srv/GetEntityState \
  "{entity_id: 'binary_sensor.intrusion'}"
```

Réponse si pas d'alarme :

```yaml
success: true
entity_state:
  entity_id: binary_sensor.intrusion
  domain: binary_sensor
  friendly_name: Intrusion
  state: 'off'
  attributes: '{"device_class": "motion", "friendly_name": "Intrusion"}'
  area_id: ''
error_message: ''
```

Réponse si alarme déclenchée :

```yaml
success: true
entity_state:
  entity_id: binary_sensor.intrusion
  domain: binary_sensor
  friendly_name: Intrusion
  state: 'on'
  attributes: '{"device_class": "motion", "friendly_name": "Intrusion"}'
  area_id: ''
error_message: ''
```

#### Vérifier si le portail est ouvert

```bash
ros2 service call /ha_bridge/get_entity_state qbo_ha_interfaces/srv/GetEntityState \
  "{entity_id: 'binary_sensor.position_portail'}"
```

```yaml
entity_state:
  entity_id: binary_sensor.position_portail
  state: 'on'      # on = ouvert  (invert: true dans knx.yaml)
  attributes: '{"device_class": "opening"}'
```

#### Vérifier l'alarme gel (protection installation)

```bash
ros2 service call /ha_bridge/get_entity_state qbo_ha_interfaces/srv/GetEntityState \
  "{entity_id: 'binary_sensor.alarme_gel'}"
```

```yaml
entity_state:
  entity_id: binary_sensor.alarme_gel
  state: 'off'
  attributes: '{"device_class": "cold", "friendly_name": "Alarme gel"}'
```

---

### Cas d'erreur

#### Entité inconnue

```bash
ros2 service call /ha_bridge/get_entity_state qbo_ha_interfaces/srv/GetEntityState \
  "{entity_id: 'light.chambre_inexistante'}"
```

```yaml
success: false
entity_state:
  entity_id: ''
  state: ''
error_message: "Entity 'light.chambre_inexistante' not found"
```

#### Home Assistant non joignable

```yaml
success: false
result: ''
error_message: 'Not connected to Home Assistant'
```

#### Timeout HA (> 15 s sans réponse)

```yaml
success: false
result: ''
error_message: 'concurrent.futures.TimeoutError'
```

---

## Intégration dans un nœud Qbo (Python)

Exemple minimal d'un nœud Qbo qui écoute un état et appelle un service :

```python
import rclpy
from rclpy.node import Node
from qbo_ha_interfaces.msg import EntityState
from qbo_ha_interfaces.srv import CallHaService


class QboDecisionNode(Node):
    def __init__(self):
        super().__init__("qbo_decision")
        self._sub = self.create_subscription(
            EntityState, "/ha_bridge/entity_state", self._on_state, 50
        )
        self._cli = self.create_client(CallHaService, "/ha_bridge/call_service")

    def _on_state(self, msg: EntityState) -> None:
        # Exemple : éteindre le salon quand la luminosité dépasse 50 000 lx
        if msg.entity_id == "sensor.luminosite_exterieure" and float(msg.state) > 50000:
            self._send_command("cover", "set_cover_position",
                               "cover.volet_roulant_sw_salon", '{"position": 20}')

    def _send_command(self, domain, service, entity_id, service_data=""):
        req = CallHaService.Request()
        req.domain = domain
        req.service = service
        req.entity_id = entity_id
        req.service_data = service_data
        self._cli.call_async(req)  # fire-and-forget depuis un callback
```
