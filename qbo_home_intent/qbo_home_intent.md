# qbo_home_intent — Guide de référence complet

## 1. Ordre d'exécution dans le nœud `/home_intent`

### 1.1 Vue d'ensemble

```text
ros2 service call /home_intent/parse  {text, session_id, resolve, execute}
              │
              ▼
      HomeIntentNode._handle_parse()
              │
              ├─► _parse()          ← toujours exécuté
              │       │
              │       ├─ normalizer.normalize()
              │       ├─ classifier.classify()
              │       └─ slot_extractor.extract()
              │       └─► HomeIntent(status=PARSED)
              │
              ├─► _resolve()        ← si resolve=true ou execute=true
              │       │
              │       ├─ heuristique action → device_class (si vide)
              │       ├─ resolver.resolve()
              │       ├─ validator.validate_action()
              │       └─ validator.get_risk()
              │       └─► HomeIntent(status=RESOLVED | NEEDS_CLARIFICATION | REJECTED)
              │
              ├─► _execute()        ← si execute=true ET status=RESOLVED
              │       │
              │       └─ executor.execute()
              │               │
              │               └─► /ha_bridge/call_service  →  Home Assistant
              │               └─► IntentResult(status=SUCCESS | FAILED | TIMEOUT)
              │
              └─► _build_status_result()  ← si execute=false OU status≠RESOLVED
                      │
                      └─► spoken_response via response_builder
```

---

### 1.2 Étape par étape : `"allume le bureau"`

#### A — `_handle_parse` reçoit la requête

```python
request.text       = "allume le bureau"
request.session_id = "test"
request.resolve    = True
request.execute    = True
```

---

#### B — `_parse()` → `HomeIntent(status=PARSED)`

**B1 · `normalizer.normalize()`**

| Transformation   | Entrée                | Sortie                  |
|------------------|-----------------------|-------------------------|
| lower + strip    | "allume le bureau"    | "allume le bureau"      |
| contractions     | (aucune ici)          | "allume le bureau"      |
| strip accents    | (aucun ici)           | "allume le bureau"      |
| ponctuation      | (aucune ici)          | "allume le bureau"      |

Résultat : `"allume le bureau"`

**B2 · `classifier.classify()`**

1. Regex `_CANCEL_RE` → pas de match
2. Regex `_STATUS_RE` → pas de match
3. Regex `_QUERY_RE` → pas de match
4. Intersection avec `_control_verbs` : `"allume"` ∈ verbs de `turn_on` → **match**

Résultat : `INTENT_CONTROL (1)`

**B3 · `slot_extractor.extract()`**

| Slot          | Recherche                          | Résultat      |
|---------------|------------------------------------|---------------|
| `action`      | fenêtres 1→6 mots dans action_map  | `turn_on`     |
| `area`        | alias → area_id (longest match)    | `bureau`      |
| `device_class`| alias devices (longest match)      | `""` (vide)   |
| `entity_id`   | alias sensors (longest match)      | `""` (vide)   |
| `parameters`  | regex numérique                    | `{}`          |

**Intent produit :**
```
status=0 (PARSED)  action='turn_on'  area='bureau'  device_class=''  confidence=0.6
```
*(confidence=0.6 car device_class vide)*

---

#### C — `_resolve()` → `HomeIntent(status=RESOLVED)`

**C1 · Heuristique action → device_class**

```
action = 'turn_on'  →  _ACTION_DEFAULT_CLASS['turn_on'] = 'light'
device_class : '' → 'light'
```

**C2 · `resolver.resolve(area='bureau', device_class='light')`**

Parcours de `home_objects.yaml` + `virtual_objects.yaml` :
```
filtre area='bureau'    → [lumiere_bureau, volet_bureau, chauffage_bureau]
filtre device_class='light' → [lumiere_bureau]
→ 1 candidat unique
```

**C3 · Hydratation de la cible**

```
intent.target.entity_id    = 'light.bureau'
intent.target.name         = 'Lumière bureau'
intent.target.device_class = 'light'
intent.target.aliases      = ['lumiere du bureau', 'eclairage bureau', 'bureau lumiere']
```

**C4 · `validator.validate_action('light', 'turn_on')`**

```yaml
# capabilities.yaml
light:
  actions: [turn_on, turn_off, set_brightness, set_color]
```
`turn_on` ∈ actions → `(True, "")`

**C5 · `validator.get_risk('light.bureau', 'light', 'turn_on', {})`**

Parcours de `safety.yaml` → aucune règle ne correspond → `risk_level=0, warning=""`

**Intent produit :**
```
status=1 (RESOLVED)  entity_id='light.bureau'  risk_level=0  confirmation_required=False
confidence=1.0
```

---

#### D — `_execute()` → `IntentResult(status=SUCCESS)`

**D1 · `executor.execute(action='turn_on', entity_id='light.bureau', device_class='light')`**

```python
ha_service   = _ACTION_TO_HA_SERVICE['turn_on']    # → 'turn_on'
domain       = _HA_DOMAIN_MAP['light']             # → 'light'
service_data = _build_service_data('turn_on', {})  # → {} (pas de paramètre)
```

**D2 · Attente de connexion ha_bridge (2 s max)**

```python
self._call_cli.wait_for_service(timeout_sec=2.0)  # → True
```

**D3 · Log et envoi**

```
[INFO] → ha_bridge  domain=light  service=turn_on  entity_id='light.bureau'  data=''
```

Requête ROS2 envoyée à `/ha_bridge/call_service` :
```
req.domain       = 'light'
req.service      = 'turn_on'
req.entity_id    = 'light.bureau'
req.service_data = ''
```

**D4 · Polling du future (10 ms, max 15 s)**

```python
while not future.done():
    time.sleep(0.01)   # MultiThreadedExecutor dispatche la réponse en parallèle
```

**D5 · Résultat**

```python
resp.success = True
spoken = _render_spoken('turn_on', 'lumiere bureau', {})
       = "J'ai allumé lumiere bureau."
```

**IntentResult produit :**
```
status=0 (SUCCESS)   success=True
spoken_response      = "J'ai allumé lumiere bureau."
technical_message    = '{"id": 42, "type": "result", "success": true, ...}'
affected_entities    = ['light.bureau']
```

---

### 1.3 Transitions de statut possibles

```text
PARSED ──(resolve)──► RESOLVED          → execute possible
                  └─► NEEDS_CLARIFICATION → spoken_response = question
                  └─► REJECTED           → spoken_response = message d'erreur

RESOLVED ──(execute)──► SUCCESS          → spoken_response = confirmation
                    └─► FAILED           → spoken_response = erreur HA
                    └─► NEEDS_CONFIRMATION → spoken_response = avertissement
                                             pending_action stocké en session
```

---

## 2. Fichiers de configuration (`config/`)

### `config/actions.yaml`

Dictionnaire maître verbes → actions. Lu par `classifier.py` et
`slot_extractor.py` au démarrage du nœud.

```yaml
control:               # actions qui modifient un état
  <action_name>:
    verbs: [...]       # liste de verbes/phrases normalisés (sans accents)
    parameters: [...]  # noms de paramètres attendus (optionnel)
    targets: [...]     # device_class ou entity_id prioritaires (optionnel)

query:                 # actions qui lisent un état
  <action_name>:
    verbs: [...]
    targets: [...]
```

**Actions de contrôle disponibles :**

| Nom               | Verbes représentatifs               | Paramètre HA           |
|-------------------|-------------------------------------|------------------------|
| `turn_on`         | allume, active, mets                | —                      |
| `turn_off`        | eteins, coupe, desactive            | —                      |
| `open`            | ouvre, leve, monte, remonte         | —                      |
| `close`           | ferme, baisse, descends             | —                      |
| `stop`            | stop, arrete                        | —                      |
| `set_brightness`  | regle la luminosite, mets a         | `brightness_pct` (0-100)|
| `set_temperature` | regle la temperature, chauffe a     | `temperature` (°C)     |
| `set_color`       | mets en, change la couleur          | `color_name`           |
| `set_position`    | positionne a, regle a               | `position` (0-100)     |
| `select_option`   | mode, passe en mode                 | `option` (texte)       |

**Actions de requête disponibles :**

| Nom                 | Verbes représentatifs               |
|---------------------|-------------------------------------|
| `read_temperature`  | quelle temperature, il fait combien |
| `read_state`        | quel est letat, est ce que          |
| `read_power`        | quelle consommation, watt           |
| `read_energy`       | combien denergie, consommation totale|

> **Pour ajouter un synonyme** : ajouter le verbe normalisé (minuscules,
> sans accents) dans la liste `verbs` de l'action concernée, puis rebuild.

---

### `config/areas.yaml`

Définit les zones de la maison. Lu par `slot_extractor.py`.

```yaml
areas:
  <area_id>:           # identifiant interne (snake_case)
    label: "..."       # nom affiché
    aliases: [...]     # alias normalisés (sans accents, minuscules)
    ha_area_id: "..."  # area_id Home Assistant correspondant
```

**Champs détaillés :**

| Champ        | Type   | Usage                                                          |
|--------------|--------|----------------------------------------------------------------|
| `area_id`    | clé    | Valeur stockée dans `HomeTarget.area`                          |
| `label`      | string | Affichage lisible                                              |
| `aliases`    | liste  | Ce que l'utilisateur peut dire (normalisé, sans accents)       |
| `ha_area_id` | string | Pour filtrage futur via l'API HA `area_id`                     |

**Zones définies :**

| area_id        | label           | Exemples d'alias              |
|----------------|-----------------|-------------------------------|
| `salon`        | Salon           | sejour, living, piece principale|
| `bureau`       | Bureau          | salle de travail, office      |
| `cuisine`      | Cuisine         | kitchenette                   |
| `chambre_1`    | Chambre 1       | chambre principale, chambre parents|
| `chambre_2`    | Chambre 2       | deuxieme chambre, chambre amis|
| `atelier`      | Atelier         | garage                        |
| `salle_a_manger`| Salle à manger | salle manger, dining          |
| `exterieur`    | Extérieur       | dehors, jardin, terrasse      |

---

### `config/aliases.yaml`

Alias pour les **types d'appareils** et les **capteurs individuels**. Lu par
`slot_extractor.py`.

```yaml
devices:
  <dev_id>:
    aliases: [...]       # alias normalisés
    device_class: "..."  # valeur pour HomeTarget.device_class
    entity_id: "..."     # si l'appareil a une entité directe (optionnel)

sensors:
  <sensor_id>:
    aliases: [...]
    entity_id: "..."     # entité HA directe
```

**Logique de résolution dans le slot extractor :**

1. Pour chaque alias (par longueur décroissante) :
   - Si alias dans le texte → `device_class` extrait
   - Si `entity_id` présent dans la définition → `entity_id` extrait directement
2. Les alias `sensors` surclassent les alias `devices` (plus spécifiques).

**Appareils définis :**

| dev_id         | device_class | entity_id directe      | Alias représentatifs       |
|----------------|--------------|------------------------|----------------------------|
| `lumiere`      | light        | —                      | lumiere, lampe, eclairage  |
| `volet`        | cover        | —                      | volet, store, rideau       |
| `chauffage`    | climate      | —                      | chauffage, thermostat      |
| `fil_pilote`   | select       | —                      | fil pilote, mode chauffage |
| `portail`      | switch       | `switch.portail`       | portail, portillon, entree |
| `porte_garage` | switch       | `switch.porte_garage`  | porte garage               |
| `cumulus`      | switch       | `switch.cumulus`       | cumulus, chauffe eau       |

**Capteurs définis :**

| sensor_id          | entity_id directe                | Alias représentatifs              |
|--------------------|----------------------------------|-----------------------------------|
| `temperature_ext`  | `sensor.temperature_exterieure`  | temperature exterieure, dehors    |
| `luminosite`       | `sensor.luminosite_exterieure`   | luminosite, ensoleillement        |
| `intrusion`        | `binary_sensor.intrusion`        | intrusion, alarme                 |
| `batterie_zendure` | `sensor.zendure_battery_level`   | batterie, zendure                 |
| `production_solaire`| `sensor.solar_production_power` | production solaire, solaire       |
| `eau_pluie`        | `sensor.citerne_eau_pluie`       | eau de pluie, citerne, cuve       |

---

### `config/capabilities.yaml`

Définit les actions autorisées par `device_class`. Lu par `validator.py`.

```yaml
capabilities:
  <device_class>:
    actions: [...]              # liste des actions de contrôle autorisées
    queryable_properties: [...] # propriétés lisibles
    read_only: true             # si true, toute action de contrôle est rejetée
    supports_dimmer: true       # information contextuelle (non utilisé en validation)
    position_range: [0, 100]    # pour couverture (non utilisé en validation)
    temperature_range: [5, 30]  # pour climate (non utilisé en validation)
```

**Tableau complet :**

| device_class    | actions autorisées                                    | read_only |
|-----------------|-------------------------------------------------------|-----------|
| `light`         | turn_on, turn_off, set_brightness, set_color          | non       |
| `cover`         | open, close, stop, set_position                       | non       |
| `climate`       | set_temperature                                       | non       |
| `switch`        | turn_on, turn_off                                     | non       |
| `select`        | select_option                                         | non       |
| `sensor`        | *(aucune)*                                            | **oui**   |
| `binary_sensor` | *(aucune)*                                            | **oui**   |

> Si `device_class` inconnu (non déclaré) : la validation passe sans erreur
> (comportement permissif pour les nouveaux types HA).

---

### `config/safety.yaml`

Règles de sécurité évaluées par `validator.get_risk()`. Lu par `validator.py`.

```yaml
risk_levels:
  <niveau>:
    label: "..."
    requires_confirmation: true|false
    verbal_warning: true         # optionnel — pour niveau 3

rules:
  - id: "..."                    # identifiant unique (debug)
    entity_id: "..."             # filtre exact sur entity_id (vide = tous)
    domain: "..."                # filtre sur le domaine HA (optionnel)
    action: "..."                # filtre sur l'action interne (optionnel)
    condition:                   # filtre numérique sur un paramètre (optionnel)
      parameter: "temperature"
      operator: ">"
      value: 27.0
    risk_level: 0|1|2|3
    warning: "..."               # phrase à vocaliser si confirmation requise
```

**Règles configurées :**

| id                  | Déclencheur                         | Niveau | Confirmation |
|---------------------|-------------------------------------|--------|--------------|
| `portail_open`      | turn_on sur switch.portail          | 2      | oui          |
| `garage_open`       | turn_on sur switch.porte_garage     | 2      | oui          |
| `cumulus_off`       | turn_off sur switch.cumulus         | 2      | oui          |
| `all_lights_off`    | turn_off sans entity_id, domaine light | 1   | non          |
| `all_covers_close`  | close sans entity_id, domaine cover | 1      | non          |
| `temperature_high`  | set_temperature > 27 °C             | 2      | oui          |
| `temperature_low`   | set_temperature < 7 °C              | 2      | oui          |

**Algorithme d'évaluation :**

La première règle dont tous les filtres correspondent est retournée.
L'ordre dans le YAML détermine la priorité.

---

## 3. Fichiers de connaissances (`knowledge/`)

### `knowledge/home_objects.yaml`

Base domestique **éditée manuellement**. C'est le fichier principal à enrichir
au fur et à mesure que de nouveaux équipements sont ajoutés.

```yaml
objects:
  - id: "..."             # identifiant unique interne (snake_case)
    label: "..."          # nom lisible (affiché dans spoken_response)
    device_class: "..."   # light | cover | climate | switch | select | sensor | binary_sensor
    area: "..."           # area_id (doit correspondre à une clé de areas.yaml)
    entity_id: "..."      # entity_id Home Assistant exact
    aliases: [...]        # alias normalisés pour la résolution par texte
    capabilities: [...]   # actions supportées (documentation, non utilisé en validation)
    risk_level: 0|1|2     # surcharge locale si différent de la règle safety.yaml
```

**Logique de résolution par le `resolver.py` :**

```
resolve(area, device_class, name, entity_id)
    │
    ├─ entity_id fourni → cherche correspondance exacte → retourne
    │
    ├─ filtre par area    (si fourni)
    ├─ filtre par device_class (si fourni)
    │
    └─ filtre par name/alias (si fourni) → retourne sous-liste
       sinon retourne tous les candidats restants
```

**Objets actuellement définis :**

```
Salon       : lumiere_salon, volet_salon, chauffage_jour, temperature_salon
Bureau      : lumiere_bureau, volet_bureau, chauffage_bureau
Chambre 1   : lumiere_chambre_1, volet_chambre_1, chauffage_chambre_1
Chambre 2   : lumiere_chambre_2, volet_chambre_2, chauffage_chambre_2
Atelier     : porte_garage, cumulus
Extérieur   : portail, batterie_zendure, production_solaire, eau_pluie,
              temperature_exterieure
```

**Pour ajouter un équipement :**

```yaml
- id: lumiere_cuisine
  label: "Lumière cuisine"
  device_class: light
  area: cuisine
  entity_id: light.cuisine
  aliases: [lumiere de la cuisine, eclairage cuisine, cuisine lumiere]
  capabilities: [turn_on, turn_off, set_brightness]
```

Rebuild nécessaire (`colcon build --packages-select qbo_home_intent`) pour
déployer le YAML modifié dans `share/qbo_home_intent/knowledge/`.

---

### `knowledge/virtual_objects.yaml`

Objets **sans entité HA unique** : agrégats (toutes les lumières) ou
abstractions (mode hors-gel général). Même format que `home_objects.yaml`
avec un champ additionnel `entities` pour les agrégats.

```yaml
virtual_objects:
  - id: "..."
    label: "..."
    device_class: "..."
    area: ""            # vide = pas de zone spécifique
    entity_id: ""       # vide = action sur le domaine entier via HA
    entities:           # liste d'entités pour les actions groupées (optionnel)
      - select.chauffage_chambre_1
      - select.chauffage_chambre_2
    aliases: [...]
    capabilities: [...]
    default_option: "..." # valeur par défaut pour select_option (optionnel)
```

**Objets virtuels définis :**

| id                  | Déclencheur                         | Comportement HA                         |
|---------------------|-------------------------------------|-----------------------------------------|
| `all_lights`        | "toutes les lumieres"               | turn_on/off domaine light entier        |
| `all_covers`        | "tous les volets"                   | open/close domaine cover entier         |
| `heating_frost_all` | "mode hors gel", "hors gel partout" | select_option sur 4 entités chauffage   |
| `comfort_mode_all`  | "mode confort partout"              | select_option Confort sur 4 entités     |

> **Limitation actuelle** : l'executor envoie une seule commande HA avec
> `entity_id=''`. Pour les agrégats multi-entités (`entities`), le support
> d'itération n'est pas encore implémenté (Étape 4).

---

### `knowledge/ha_entities.json`

Cache des entités importées automatiquement depuis les registres Home Assistant.
Actuellement vide — sera peuplé à l'**Étape 5** via `get_all_states`.

```json
{
  "generated_at": "2026-08-06T00:00:00",
  "note": "Populated automatically from HA get_all_states (Étape 5).",
  "entities": []
}
```

Format cible (Étape 5) :

```json
{
  "entities": [
    {
      "entity_id": "light.atelier",
      "domain": "light",
      "friendly_name": "Atelier",
      "area_id": "atelier",
      "state": "off"
    }
  ]
}
```

---

## 4. Fichiers de tests (`tests/`)

Les fichiers de test sont au format YAML. Ils servent de **référence de
régression** : pour chaque phrase, on définit le résultat attendu après parse
et resolve (sans exécution).

Structure commune :

```yaml
tests:
  - text: "..."          # phrase en français (non normalisée)
    expected:
      intent_type: control|query|status|cancel
      action: "..."      # action interne attendue
      area: "..."        # area_id attendu (optionnel)
      device_class: "..."
      entity_id: "..."   # si résolution directe vers une entité
      status: resolved|needs_clarification|rejected
      missing_slot: "..."
      parameters:
        <key>: "<value>"
      risk_level: 0|1|2|3
      confirmation_required: true|false
      warning: "..."
      note: "..."        # explication (non utilisé par le runner)
```

---

### `tests/control_commands.yaml`

Commandes de contrôle directes. Tous les cas doivent atteindre
`status=resolved` après resolve.

**Cas couverts :**

| Phrase                                  | action           | area    | device_class |
|-----------------------------------------|------------------|---------|--------------|
| "allume le salon"                       | turn_on          | salon   | light        |
| "éteins la lumière du bureau"           | turn_off         | bureau  | light        |
| "ferme les volets du salon"             | close            | salon   | cover        |
| "ouvre les volets du bureau"            | open             | bureau  | cover        |
| "règle la luminosité du salon à 40%"   | set_brightness   | salon   | light        |
| "mets le chauffage à 21 degrés"         | set_temperature  | —       | climate      |
| "éteins toutes les lumières"            | turn_off         | —       | light        |
| "arrête les volets"                     | stop             | —       | cover        |
| "positionne le volet salon à 30%"       | set_position     | salon   | cover        |
| "active le cumulus"                     | turn_on          | —       | — (entity_id direct)|

---

### `tests/information_queries.yaml`

Requêtes d'information. Doivent atteindre `intent_type=query` et résoudre
vers le bon capteur.

**Cas couverts :**

| Phrase                                        | action           | entity_id / area          |
|-----------------------------------------------|------------------|---------------------------|
| "quelle température fait-il dans le salon"    | read_temperature | sensor.thermometre_salon… |
| "quelle est la température extérieure"        | read_temperature | sensor.temperature_ext…   |
| "combien fait-il dehors"                      | —                | sensor.temperature_ext…   |
| "est-ce que le salon est allumé"              | read_state       | area=salon, device=light  |
| "quel est le niveau de la batterie Zendure"   | —                | sensor.zendure_battery…   |
| "quelle est la production solaire"            | —                | sensor.solar_production…  |
| "quel est l'état du portail"                  | —                | binary_sensor.position_portail|
| "combien y a-t-il d'eau de pluie"             | —                | sensor.citerne_eau_pluie  |

---

### `tests/ambiguous_commands.yaml`

Commandes incomplètes. Doivent déclencher `status=needs_clarification` avec
le `missing_slot` correct.

**Cas couverts :**

| Phrase                     | missing_slot | Raison                                    |
|----------------------------|--------------|-------------------------------------------|
| "allume la lumière"        | area         | Plusieurs lumières sans zone              |
| "ferme les volets"         | area         | Plusieurs volets sans zone                |
| "règle le chauffage"       | value        | Consigne de température manquante         |
| "éteins"                   | —            | Ni objet ni zone                          |
| "mets à 50"                | —            | Valeur présente mais action/objet ambigus |

---

### `tests/safety_commands.yaml`

Commandes à risque. Vérifient que les règles de `safety.yaml` sont correctement
appliquées.

**Cas couverts :**

| Phrase                           | risk_level | confirmation_required | warning                                      |
|----------------------------------|------------|-----------------------|----------------------------------------------|
| "ouvre le portail"               | 2          | true                  | "Je vais ouvrir le portail. Confirmez-vous ?" |
| "ouvre la porte du garage"       | 2          | true                  | "Je vais ouvrir la porte du garage…"          |
| "éteins toutes les lumières"     | 1          | false                 | "Je vais éteindre toutes les lumières."       |
| "coupe le cumulus"               | 2          | true                  | "Je vais couper le cumulus. Confirmez-vous ?" |
| "règle le chauffage à 30 degrés" | 2          | true                  | "La consigne demandée est élevée…"            |
| "règle le chauffage à 5 degrés"  | 2          | true                  | "La consigne est très basse…"                 |

---

## 5. Heuristique action → device_class

Quand `device_class` est vide après l'extraction de slots (l'utilisateur n'a
pas nommé le type d'appareil), le nœud infère le type depuis l'action :

```python
_ACTION_DEFAULT_CLASS = {
    "turn_on"        : "light",   # "allume le bureau" → cherche light dans bureau
    "turn_off"       : "light",   # "éteins le salon"  → cherche light dans salon
    "set_brightness" : "light",
    "set_color"      : "light",
    "open"           : "cover",   # "ouvre dans le bureau" → cherche cover dans bureau
    "close"          : "cover",
    "stop"           : "cover",
    "set_position"   : "cover",
    "set_temperature": "climate",
    "select_option"  : "select",
    "read_temperature": "sensor",
    "read_power"     : "sensor",
    "read_energy"    : "sensor",
}
```

> Si l'utilisateur dit "allume le bureau" → `device_class=light` → resolver
> filtre sur `(area=bureau, device_class=light)` → `light.bureau`.
>
> Si l'utilisateur dit "ouvre le salon" → `device_class=cover` → resolver
> filtre sur `(area=salon, device_class=cover)` → `cover.volet_roulant_sw_salon`.

---

## 6. Concurrence et gestion des timeouts

### Problème

`rclpy.spin_until_future_complete()` depuis un callback de service bloque
l'unique thread de l'exécuteur → la réponse du client ha_bridge n'est jamais
traitée → timeout systématique (la lumière s'allume quand même car HA reçoit
la commande via WebSocket avant que le timeout ne soit détecté).

### Solution implémentée

```
main.py
  └─ MultiThreadedExecutor.spin()
       ├─ Thread 1 : callback _handle_parse  (ReentrantCallbackGroup)
       │    └─ executor._wait(future)  ← polling 10 ms
       │                                  (ne bloque pas d'autres threads)
       └─ Thread 2 : callback réponse ha_bridge
            └─ future.set_result()  ← Thread 1 sort du polling
```

`ReentrantCallbackGroup` est nécessaire sur les services **et** les clients
pour que leurs callbacks puissent s'exécuter en parallèle dans le même
`MultiThreadedExecutor`.

---

## 7. Récapitulatif des statuts

### `HomeIntent.status` (après `_resolve`)

| Valeur | Constante                  | Que faire côté appelant                        |
|--------|----------------------------|------------------------------------------------|
| 0      | `STATUS_PARSED`            | Slots extraits, pas encore résolu              |
| 1      | `STATUS_RESOLVED`          | Prêt pour `_execute`                           |
| 2      | `STATUS_NEEDS_CLARIFICATION`| Vocaliser `result.spoken_response` (question) |
| 3      | `STATUS_REJECTED`          | Vocaliser `result.spoken_response` (erreur)    |

### `IntentResult.status` (après `_execute` ou `_build_status_result`)

| Valeur | Constante                   | Exemple de `spoken_response`                     |
|--------|-----------------------------|--------------------------------------------------|
| 0      | `STATUS_SUCCESS`            | « J'ai allumé lumiere bureau. »                  |
| 1      | `STATUS_FAILED`             | « Je ne peux pas accéder à la maison... »        |
| 2      | `STATUS_CANCELLED`          | (annulation en cours de confirmation)            |
| 3      | `STATUS_NEEDS_CONFIRMATION` | « Je vais ouvrir le portail. Confirmez-vous ? »  |
| 4      | `STATUS_NEEDS_CLARIFICATION`| « Dans quelle pièce ? »                          |

---

## 8. Commandes de lancement

### 8.1 Nœud principal `home_intent`

```bash
# Lancement standard
ros2 launch qbo_home_intent home_intent.launch.py

# Avec niveau de log verbeux
ros2 launch qbo_home_intent home_intent.launch.py log_level:=debug
```

---

### 8.2 Reconnaissance vocale Whisper (`qbo_driver`)

```bash
# Lance le node qbo_listen (STT Whisper) — publie sur /listen
ros2 launch qbo_driver voice_input.launch.py
```

---

### 8.3 Test runner (`runner_tests`)

```bash
# Tous les fichiers YAML de tests/
ros2 run qbo_home_intent runner_tests

# Un ou plusieurs fichiers spécifiques
ros2 run qbo_home_intent runner_tests control_commands.yaml
ros2 run qbo_home_intent runner_tests safety_commands.yaml ambiguous_commands.yaml

# Mode vocal live : écoute /listen, exécute et répond via TTS
ros2 run qbo_home_intent runner_tests --live
```

Mode `--live` : utilise un `MultiThreadedExecutor` + `ReentrantCallbackGroup` ;
les commandes sont exécutées (`execute=true`) et la réponse est envoyée
au service `/qbo_driver/say_to_TTS`.

---

### 8.4 Synchronisation entités HA (`ha_entities_sync`)

```bash
# Import complet → écrit ha_entities.json
ros2 run qbo_home_intent ha_entities_sync

# Afficher uniquement un domaine (pas d'écriture JSON)
ros2 run qbo_home_intent ha_entities_sync -d light
ros2 run qbo_home_intent ha_entities_sync -d cover
ros2 run qbo_home_intent ha_entities_sync -d select

# Plusieurs domaines simultanément
ros2 run qbo_home_intent ha_entities_sync -d light cover switch

# Filtrer par area_id HA (si assigné dans l'interface HA)
ros2 run qbo_home_intent ha_entities_sync -a bureau

# Combiner domaine + pièce
ros2 run qbo_home_intent ha_entities_sync -d light -a salon

# Sortie JSON dans un chemin personnalisé
ros2 run qbo_home_intent ha_entities_sync -o /tmp/ha_dump.json
```

> En mode filtré (`-d` ou `-a`), le fichier `ha_entities.json` n'est **pas** écrit.
> Relancer sans filtre pour une synchro complète.

---

### 8.5 Appels directs au service `/home_intent/parse`

```bash
# Parse + resolve uniquement (pas d'exécution HA)
ros2 service call /home_intent/parse qbo_home_interfaces/srv/ParseHomeCommand \
  "{text: 'allume le bureau', session_id: 'test', resolve: true, execute: false}"

# Parse + resolve + execute (action réelle sur HA)
ros2 service call /home_intent/parse qbo_home_interfaces/srv/ParseHomeCommand \
  "{text: 'allume le bureau', session_id: 'test', resolve: true, execute: true}"

# Requête d'information
ros2 service call /home_intent/parse qbo_home_interfaces/srv/ParseHomeCommand \
  "{text: 'quelle température fait-il dehors', session_id: 'test', resolve: true, execute: false}"
```

---

### 8.6 Inspection des topics ROS2

```bash
# Vérifier la transcription Whisper
ros2 topic echo /listen

# Lister les nœuds actifs
ros2 node list | grep -E "home_intent|ha_bridge|qbo"

# Vérifier les services disponibles
ros2 service list | grep home_intent
```

---

### 8.7 Ordre de démarrage recommandé

```
1.  ha_bridge          (passerelle Home Assistant)
2.  home_intent        ros2 launch qbo_home_intent home_intent.launch.py
3a. voice_input        ros2 launch qbo_driver voice_input.launch.py
 OU
3b. runner_tests live  ros2 run qbo_home_intent runner_tests --live
 OU
3c. runner_tests yaml  ros2 run qbo_home_intent runner_tests
```
