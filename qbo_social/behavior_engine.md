# QBO Social Behavior Engine

**Version:** 2.1
**Node:** `qbo_social_behavior_engine`

---

## 📋 Table des matières

1. [Objectif](#objectif)
2. [Ce que le Behavior Engine NE DOIT PAS faire](#ce-que-le-behavior-engine-ne-doit-pas-faire)
3. [Paramètres configurables](#paramètres-configurables)
4. [Schéma de décision](#schéma-de-décision)
5. [Système de mémoire](#système-de-mémoire)
6. [Traces de décision](#traces-de-décision)

---

## 🎯 Objectif

Le **Behavior Engine** est le cerveau décisionnel du système social de QBO. Son rôle est de :

1. **Analyser les événements** sociaux entrants (`PERSON_RECOGNIZED`, `FACE_STABLE`, `FACE_LOST`)
2. **Consulter l'état du monde** fourni par le World Model (`face_stable`, `face_present`, etc.)
3. **Prendre des décisions** intelligentes basées sur plusieurs niveaux de mémoire
4. **Publier des intentions** (`GREET_PERSON`, etc.) qui seront exécutées par l'Action Executor
5. **Tracer toutes les décisions** pour permettre le debug et l'analyse du comportement

### Principe de fonctionnement

Le Behavior Engine implémente une **logique de salutation intelligente** avec quatre niveaux de protection :

- **Mémoire de session** : évite de saluer plusieurs fois la même personne dans une session continue
- **Mémoire sociale longue** : évite de re-saluer une personne qui part et revient rapidement
- **Mémoire technique courte** : lie temporellement la reconnaissance faciale et la stabilité du visage
- **Mémoire spatiale** : vérifie la continuité de position pour éviter les fausses reconnaissances

---

## 🚫 Ce que le Behavior Engine NE DOIT PAS faire

Le Behavior Engine est un **moteur de décision pur**. Il ne doit **JAMAIS** :

❌ **Exécuter directement des actions**
→ Il publie des intentions, c'est l'Action Executor qui les exécute

❌ **Agréger ou modifier l'état du monde**
→ C'est le rôle du World Model

❌ **Interpréter des données brutes de capteurs**
→ C'est le rôle de l'Event Adapter

❌ **Gérer les modes système (NORMAL, STANDBY, etc.)**
→ C'est le rôle du System Mode Manager

❌ **Arbitrer entre plusieurs intentions contradictoires**
→ C'est le rôle de l'Intent Arbiter (si implémenté)

✅ **Il doit uniquement** : recevoir des événements → analyser → décider → publier une intention

---

## ⚙️ Paramètres configurables

Tous les paramètres sont configurables via le fichier `qbo_social_params.yaml` ou en ligne de commande.

### `greet_memory_duration`

**Type:** `float`
**Valeur par défaut:** `60.0` secondes
**Description:** Durée de la mémoire sociale longue. Temps pendant lequel le robot se souvient d'avoir salué une personne spécifique. Si la personne part et revient avant l'expiration de ce délai, elle ne sera **pas** saluée à nouveau.

**Cas d'usage:**
- `20.0s` : Mode accueil (re-salue rapidement)
- `60.0s` : Mode normal (équilibre)
- `300.0s` : Mode surveillance (salue rarement)

---

### `recognition_memory_duration`

**Type:** `float`
**Valeur par défaut:** `3.0` secondes
**Description:** Fenêtre temporelle pour lier un événement `PERSON_RECOGNIZED` et un événement `FACE_STABLE`. Si `FACE_STABLE` arrive plus de `recognition_memory_duration` secondes après la reconnaissance, la salutation est **ignorée**.

**Cas d'usage:**
- `2.0s` : Stricte (reconnaissance et stabilité doivent être quasi-simultanées)
- `3.0s` : Normal (permet un petit délai)
- `5.0s` : Souple (utile pour debug ou visages difficiles à stabiliser)

---

### `greet_cooldown`

**Type:** `float`
**Valeur par défaut:** `10.0` secondes
**Description:** *Actuellement non utilisé dans la logique principale* (garde-fou global optionnel). Peut servir de limite absolue entre deux salutations quelconques.

**Note:** La vraie protection contre le spam est gérée par `greet_memory_duration` (par personne).

---

### `session_timeout`

**Type:** `float`
**Valeur par défaut:** `10.0` secondes
**Description:** Timeout de session après un événement `FACE_LOST`. Si le visage revient avant ce délai, la session est **préservée**. Au-delà, la session est **resetée**. Permet de gérer les pertes courtes (personne qui tourne la tête, obstruction temporaire).

**Cas d'usage:**
- `5.0s` : Strict (reset rapide)
- `10.0s` : Normal (tolère pertes courtes)
- `15.0s` : Souple (tolère absences plus longues)

---

### `lost_context_memory_duration`

**Type:** `float`
**Valeur par défaut:** `5.0` secondes
**Description:** Durée pendant laquelle le système garde en mémoire la position et l'identité de la personne perdue pour vérifier la **continuité spatiale** lors d'une nouvelle détection. Empêche les fausses reconnaissances.

**Cas d'usage:**
- `3.0s` : Court (validation stricte)
- `5.0s` : Normal (équilibre)
- `8.0s` : Long (plus de tolérance)

---

### `spatial_reassociation_threshold`

**Type:** `float`
**Valeur par défaut:** `0.8` mètres
**Description:** Distance euclidienne maximale (en mètres) entre la position de perte et la nouvelle position pour considérer qu'il s'agit de la **même personne**. Coordonnées en repère `base_link` (monde), stable même si tête/base bougent.

**Cas d'usage:**
- `0.5m` : Très strict (environnement statique)
- `0.8m` : Normal (bon compromis)
- `1.2m` : Souple (environnement dynamique)

**Important:** Si une reconnaissance arrive avec une position incompatible (> seuil) par rapport au contexte de perte, elle est **rejetée** avec trace `spatial_discontinuity_detected`.
```
┌─────────────────────────────────────────────────────────┐
│                    BEHAVIOR ENGINE                      │
│                                                         │
│  Inputs:                                                │
│  ┌──────────────────┐        ┌──────────────────┐       │
│  │ SocialEvent      │        │ WorldState       │       │
│  │ /qbo_social/     │        │ /qbo_social/     │       │
│  │ events           │        │ world_state      │       │
│  └──────────────────┘        └──────────────────┘       │
│                                                         │
│  Outputs:                                               │
│  ┌──────────────────┐        ┌──────────────────┐       │
│  │ BehaviorIntent   │        │ DecisionTrace    │       │
│  │ /qbo_social/     │        │ /qbo_social/     │       │
│  │ intent_raw       │        │ decision_trace   │       │
│  └──────────────────┘        └──────────────────┘       │
└─────────────────────────────────────────────────────────┘
```

---

### Logique de décision détaillée

#### 1️⃣ **Événement : `PERSON_RECOGNIZED`**

```
┌─────────────────────────────────────────────────────────────────┐
│ PERSON_RECOGNIZED                                               │
└─────────────────────────────────────────────────────────────────┘
                           ↓
                   ┌───────────────┐
                   │ person_name   │
                   │ vide ?        │
                   └───────────────┘
                      ↓YES       ↓NO
        ┌─────────────┘           └──────────────┐
        │                                        │
   TRACE: ignored_                    ┌──────────────────────────┐
   empty_person_name                  │ VÉRIFICATION SPATIALE    │
   relevance=0.0                      │ _check_spatial_continuity│
                                      └──────────────────────────┘
                                         ↓FAIL          ↓PASS
                          TRACE: spatial_               │
                          discontinuity_detected        │
                          relevance=0.6           Mémoriser:
                          → REJET                 - last_recognized_person
                                                  - last_recognized_time
                                                  - last_face_position
                                                        │
                                                  TRACE: person_recognized_memorized
                                                  relevance=0.5
                                                        │
                                                        ↓
                                           ┌────────────────────────┐
                                           │ face_stable déjà TRUE? │
                                           │ (dans WorldState)      │
                                           └────────────────────────┘
                                                ↓YES          ↓NO
                                   ┌────────────┘              └─────────┐
                                   │                                     │
                          _try_greet_with_                           Attendre
                          recent_recognition()                       FACE_STABLE
```

---

#### 2️⃣ **Événement : `FACE_STABLE`**

```
┌─────────────────────────────────────────────────────────────────┐
│ FACE_STABLE                                                     │
└─────────────────────────────────────────────────────────────────┘
                           ↓
            Sauvegarder last_face_position
                           ↓
              ┌────────────────────────┐
              │ face_lost_at existe ?  │
              │ (perte récente)        │
              └────────────────────────┘
                  ↓NON           ↓OUI
                  │         ┌──────────────────────┐
                  │         │ loss_duration        │
                  │         │ < session_timeout ?  │
                  │         └──────────────────────┘
                  │            ↓NON         ↓OUI
                  │    Reset session   Préserver session
                  │    TRACE: long_    TRACE: session_
                  │    loss_X.Xs       preserved_X.Xs
                  │            │              │
                  └────────────┴──────────────┘
                              ↓
              _try_greet_with_recent_recognition()
```

---

#### 3️⃣ **Fonction : `_try_greet_with_recent_recognition()`**

Cette fonction effectue **6 vérifications** avant d'autoriser une salutation :

```
┌───────────────────────────────────────────────────────────────────┐
│ _try_greet_with_recent_recognition()                              │
└───────────────────────────────────────────────────────────────────┘
                              ↓
           ┌─────────────────────────────────────┐
           │ ❶ Personne reconnue en mémoire ?    │
           │   (last_recognized_person != "")    │
           └─────────────────────────────────────┘
                 ↓NO                    ↓YES
    TRACE: ignored_no_                  │
    recognized_person                   ↓
    relevance=0.1              ┌──────────────────────────────────┐
                               │ ❷ Reconnaissance récente ?       │
                               │   (now - last_recognized_time)   │
                               │   < recognition_memory_duration  │
                               └──────────────────────────────────┘
                                   ↓NO                  ↓YES
                      TRACE: ignored_                   │
                      recognition_too_old               ↓
                      relevance=0.2          ┌─────────────────────────┐
                                             │ ❸ Visage stable ?       │
                                             │   (world.face_stable)   │
                                             └─────────────────────────┘
                                                 ↓NO           ↓YES
                                    TRACE: ignored_            │
                                    face_not_stable            ↓
                                    relevance=0.2   ┌──────────────────────────┐
                                                    │ ❹ Déjà salué en session? │
                                                    │   (session_greeted &&    │
                                                    │   session_person_name==) │
                                                    └──────────────────────────┘
                                                        ↓YES          ↓NO
                                          TRACE: suppressed_           │
                                          already_greeted_in_session   ↓
                                          relevance=0.3    ┌──────────────────────────┐
                                          suppressed=True  │ ❺ Déjà salué récemment ? │
                                                           │  (last_greet_by_person)  │
                                                           │  < greet_memory_duration │
                                                           └──────────────────────────┘
                                                              ↓YES           ↓NO
                                                 TRACE: suppressed_          │
                                                 already_greeted_recently    │
                                                 relevance=0.4               │
                                                 suppressed=True             │
                                                                             ↓
                                                           ┌─────────────────────────────┐
                                                           │ ✅ SALUTATION AUTORISÉE
                                                           └─────────────────────────────┘
                                                                             ↓
                                                ┌────────────────────────────────────────┐
                                                │ Publier INTENT: GREET_PERSON           │
                                                │ TRACE: face_stable_with_recent_recog   │
                                                │ relevance=0.9, suppressed=False        │
                                                └────────────────────────────────────────┘
                                                                             ↓
                                                ┌────────────────────────────────────────┐
                                                │ Mise à jour des mémoires :             │
                                                │ - session_person_name                  │
                                                │ - session_greeted = True               │
                                                │ - last_greet_by_person[person] = now   │
                                                │ - Cleanup mémoire longue               │
                                                │ - Reset mémoire technique courte       │
                                                └────────────────────────────────────────┘
```

---

#### 4️⃣ **Événement : `FACE_LOST`**

```
┌─────────────────────────────────────────────────────────────────┐
│ FACE_LOST                                                       │
└─────────────────────────────────────────────────────────────────┘
                              ↓
                  Calculer session_duration
                              ↓
        TRACE: session_reset_duration_XX.Xs
        relevance=0.0
                              ↓
        ┌────────────────────────────────────────┐
        │ Reset MÉMOIRE TECHNIQUE & SESSION :    │
        │ - session_person_name = ""             │
        │ - session_greeted = False              │
        │ - session_started_at = None            │
        │ - last_recognized_person = ""          │
        │ - last_recognized_time = 0.0           │
        │                                        │
        │ ⚠️ CONSERVATION MÉMOIRE SOCIALE :
        │ - last_greet_by_person (PRÉSERVÉ)
        └────────────────────────────────────────┘
```

**Important :** La mémoire sociale (`last_greet_by_person`) est **préservée** pour éviter de re-saluer une personne qui part et revient rapidement.

---

## 💾 Système de mémoire

Le Behavior Engine utilise **quatre niveaux de mémoire** pour des décisions intelligentes :

### 1. Mémoire technique courte (volatile)

**Durée de vie :** Quelques secondes
**Objectif :** Lier temporellement reconnaissance et stabilité
**Variables :**
- `last_recognized_person` : Nom de la dernière personne reconnue
- `last_recognized_time` : Timestamp de la reconnaissance

**Reset :** Après salutation OU sur `FACE_LOST`

---

### 2. Mémoire de session (scope: une session)

**Durée de vie :** Variable (jusqu'à `session_timeout` après `FACE_LOST`)
**Objectif :** Éviter de saluer plusieurs fois dans une session continue
**Variables :**
- `session_greeted` : Booléen indiquant si on a déjà salué
- `session_person_name` : Nom de la personne saluée
- `session_started_at` : Timestamp de début de session (lors de la salutation)
- `face_lost_at` : Timestamp de perte de visage

**Reset :** Sur timeout (`session_timeout` secondes après `FACE_LOST`) ou perte longue

---

### 3. Mémoire sociale longue (persistante)

**Durée de vie :** `greet_memory_duration` secondes (par défaut 60s)
**Objectif :** Éviter de re-saluer une personne qui part et revient
**Variables :**
- `last_greet_by_person` : Dictionnaire `{"nom": timestamp}`

**Reset :** Nettoyage automatique des entrées obsolètes
**Conservation :** **PRÉSERVÉE** sur `FACE_LOST` et reset de session

---

### 4. Mémoire spatiale (continuité)

**Durée de vie :** `lost_context_memory_duration` secondes (par défaut 5s)
**Objectif :** Vérifier la plausibilité de continuité entre perte et nouvelle détection
**Variables :**
- `last_face_position` : Position 3D du dernier visage stable/reconnu `{x, y, z, distance}`
- `pending_lost_context` : Contexte de perte `{"person_name", "lost_at", "position"}`

**Fonction :** `_check_spatial_continuity()` calcule la distance euclidienne entre ancienne et nouvelle position. Si > `spatial_reassociation_threshold`, la reconnaissance est **rejetée**.

**Reset :** Sur expiration du timeout ou confirmation de continuité

---

### Tableau récapitulatif

| Mémoire                  | Durée    | Reset sur FACE_LOST | Objectif                                    |
|--------------------------|----------|---------------------|---------------------------------------------|
| **Technique courte**     | ~3s      | ✅ OUI               | Lier reconnaissance et stabilité            |
| **Session**              | ~10s     | ⏱️ TIMEOUT           | Éviter salutations multiples dans 1 session |
| **Sociale longue**       | ~60s     | ❌ NON               | Éviter re-salutation rapide après départ    |
| **Spatiale**             | ~5s      | ❌ NON               | Vérifier continuité de position             |

## 📊 Traces de décision

Chaque décision publie une trace sur `/qbo_social/decision_trace` avec :

| Champ               | Description                                              |
|---------------------|----------------------------------------------------------|
| `triggering_event`  | Type d'événement déclencheur                             |
| `chosen_intent`     | Intention choisie ("GREET_PERSON" ou "")                 |
| `reason`            | Raison de la décision (clé pour le debug)                |
| `relevance_score`   | Score de pertinence (0.0 - 1.0)                          |
| `suppressed`        | `True` si l'action a été supprimée par un cooldown       |

### Raisons possibles

**Salutation ignorée :**
- `ignored_empty_person_name` - Nom vide
- `ignored_no_recognized_person` - Aucune personne en mémoire
- `ignored_recognition_too_old` - Reconnaissance expirée
- `ignored_face_not_stable` - Visage non stable

**Salutation supprimée :**
- `suppressed_already_greeted_in_session` - Déjà salué dans cette session
- `suppressed_already_greeted_recently` - Déjà salué récemment (mémoire longue)
- `spatial_discontinuity_detected_dist_X.XXm_time_Y.Ys` - Discontinuité spatiale détectée

**Salutation autorisée :**
- `face_stable_with_recent_recognition` - Toutes conditions OK
- `spatial_continuity_confirmed_dist_X.XXm_time_Y.Ys` - Continuité spatiale confirmée

**Session :**
- `person_recognized_memorized` - Reconnaissance mémorisée
- `face_lost_session_pending` - Visage perdu, session en attente de timeout
- `session_preserved_short_loss_X.Xs` - Session préservée après perte courte
- `long_loss_X.Xs_duration_Y.Ys` - Reset après perte longue
- `session_timeout_expired_duration_X.Xs` - Reset par timeout automatique

---

## 🔧 Utilisation

### Lancement

```bash
ros2 run qbo_social behavior_engine
```

### Avec paramètres personnalisés

```bash
ros2 run qbo_social behavior_engine --ros-args \
  -p greet_memory_duration:=30.0 \
  -p recognition_memory_duration:=5.0 \
  -p session_timeout:=15.0 \
  -p spatial_reassociation_threshold:=1.0
```

### Debug des traces

```bash
ros2 topic echo /qbo_social/decision_trace
```

---

## 📝 Notes de version

### Version 2.1 (Avril 2026)

**Nouveautés majeures :**
- ✅ **Mémoire spatiale** : vérification de continuité de position pour éviter fausses reconnaissances
- ✅ **Timeout de session intelligent** : préservation de session lors de pertes courtes
- ✅ **Timer de vérification** : reset automatique de session après timeout
- ✅ Nouveaux paramètres : `session_timeout`, `lost_context_memory_duration`, `spatial_reassociation_threshold`

**Améliorations :**
- Détection des discontinuités spatiales (personne qui part, autre qui arrive)
- Contexte de perte sauvegardé avec position 3D
- Traces enrichies avec distances et durées
- Session démarrée au moment de la salutation (plus précis)
- Warning logs quand discontinuité spatiale détectée

**Correctifs :**
- Fix : fausses reconnaissances lors de changements rapides de personnes
- Fix : session_started_at initialisé au bon moment

---

### Version 2.0 (Avril 2026)

**Nouveautés :**
- ✅ Système de mémoire à trois niveaux
- ✅ Cooldown par personne (mémoire sociale)
- ✅ Séparation mémoire session / mémoire sociale
- ✅ Traces de décision exhaustives
- ✅ Paramètres configurables via YAML

**Améliorations :**
- Logique de salutation plus robuste
- Prévention du spam de salutations
- Conservation mémoire sociale sur FACE_LOST

---

**Auteur :** QBO Social Team
**Licence :** Propriétaire
**Contact :** [votre-contact]
