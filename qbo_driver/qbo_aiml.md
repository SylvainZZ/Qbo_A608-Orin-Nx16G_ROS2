# qbo_aiml — Documentation complète

## 1. Vue d'ensemble

`qbo_aiml` est le moteur de dialogue du robot Qbo. Il combine un index
vectoriel FAISS (RAG — Retrieval-Augmented Generation) et un LLM léger
(Qwen2-0.5B) pour répondre à des phrases en langage naturel et signaler
des événements de diagnostic matériel.

```text
Entrée vocale (texte)
        │
        ▼
/aiml/generate_text  {type, text, context_json}
        │
        ├─► type="conversation"
        │       ├─ QALoader.search_topk()   ← index FAISS "index_*"
        │       ├─ select_best_candidate()  ← seuil listen|dialog
        │       └─ generate_answer()        ← template + params
        │       └─► response_text (réponse vocale)
        │
        └─► type="diagnostic"
                ├─ build_diagnostic_query()
                ├─ QALoader.search_topk()   ← index FAISS "diag_*"
                ├─ _route_from_event()      ← table de routage hardware
                ├─ _select_diag_candidate() ← seuil 0.88
                └─► response_text (phrase vocalée par le robot)

/aiml/vectorize_index  (Trigger)
        └─► Reconstruit les deux index FAISS depuis data_pairs/
```

---

## 2. Services ROS2

### `/aiml/generate_text`  (`qbo_msgs/srv/GenerateText`)

Service principal — appelé par le SBE pour chaque tour de dialogue.

#### Requête

| Champ          | Type   | Valeur                                      |
|----------------|--------|---------------------------------------------|
| `type`         | string | `"conversation"` ou `"diagnostic"`          |
| `text`         | string | Texte saisi/transcrit (conversation)        |
| `context_json` | string | JSON optionnel (params SBE ou données diag) |

**Format `context_json` en mode conversation :**
```json
{
  "params": {
    "color": 2,
    "color_name": "bleu",
    "value": 42
  }
}
```
Les clés de `params` remplacent les variables `{color}`, `{value}`, … dans le
template de réponse QA.

**Format `context_json` en mode diagnostic :**
```json
{
  "key":      "Qboard_3|Battery Controller",
  "severity": "WARN",
  "message":  "battery voltage low"
}
```

#### Réponse

| Champ           | Type   | Contenu                                        |
|-----------------|--------|------------------------------------------------|
| `success`       | bool   | `false` si aucune correspondance ou erreur     |
| `response_text` | string | Phrase à vocaliser (vide si `success=false`)   |

---

### `/aiml/vectorize_index`  (`std_srvs/srv/Trigger`)

Reconstruit les deux index FAISS depuis les fichiers JSON de `data_pairs/` :

- **index** : toutes les entrées (conversation + listen)
- **diag** : uniquement `meta.intent_kind == "diagnostic"`

```bash
ros2 service call /aiml/vectorize_index std_srvs/srv/Trigger
```

---

## 3. Flux de traitement détaillé

### Mode `conversation`

```
text  →  normalize (lower + strip)
      →  QALoader.search_topk(text, k=5)
              └─ encode "query: {text}"  via e5-small-v2
              └─ FAISS inner-product search (cosine)
              └─ top-5 candidats [{item, score}, ...]
      →  select_best_candidate(candidates, text, params)
              └─ retient le score maximum
              └─ rejet si score < 0.50 (seuil minimum absolu)
      →  choix du seuil métier :
              "intent" dans item  →  THRESHOLDS["listen"]  = 0.70
              sinon               →  THRESHOLDS["dialog"]  = 0.75
      →  rejet si score < seuil
      →  generate_answer(item, None, params)
              └─ random.choice(item["answer"])
              └─ remplacement {key} → str(params[key])
      →  response_text
```

### Mode `diagnostic`

```
context_json  →  key, severity, message
              →  build_diagnostic_query()   ← texte structuré hardware/component/issue
              →  QALoader.search_topk(query, k=20)  ← index "diag_*"
              →  _route_from_event(key, message)
                      └─ parcours _ROUTE_TABLE (hw / cat / msg)
                      └─► {domain, component}
              →  _select_diag_candidate(candidates, route, threshold=0.88)
                      Passe 0 : filtre intent_kind == "diagnostic"
                      Passe 1 : filtre meta.domain == route["domain"]
                      Passe 2 : tri par score + bonus composant (+0.02)
                      Passe 3 : fallback global (seuil + 0.05)
              →  generate_answer(item, None, {})
              →  response_text
```

---

## 4. Paramètres configurables

### 4.1 Seuils de confiance FAISS

Définis directement dans `aiml.py` — **à externaliser dans un YAML** :

```python
THRESHOLDS = {
    "listen":     0.70,   # entrées QA avec un champ "intent"
    "diagnostic": 0.88,   # événements matériel
    "dialog":     0.75,   # conversation pure
}
```

| Seuil         | Effet si trop bas              | Effet si trop haut          |
|---------------|--------------------------------|-----------------------------|
| `listen`      | Faux positifs (mauvaise action)| Commandes non reconnues     |
| `diagnostic`  | Mauvais diagnostic vocalisé    | Diagnostics ignorés         |
| `dialog`      | Réponses hors-sujet            | Trop de fallback            |

Utiliser `test_engine.py --calibrate` pour trouver les valeurs optimales par domaine.

### 4.2 Modèles

| Variable          | Valeur par défaut              | Usage                        |
|-------------------|--------------------------------|------------------------------|
| `EMBED_MODEL_NAME`| `intfloat/e5-small-v2`         | Vectorisation FAISS          |
| `GEN_MODEL_NAME`  | `Qwen/Qwen2-0.5B-Instruct`     | Réécriture LLM (désactivée)  |

`enable_style_rewrite = False` → le LLM Qwen est chargé mais **jamais appelé**.
Il peut être désactivé entièrement pour économiser la RAM GPU en passant
`enable=False` dans le constructeur de `LLMEngine`.

### 4.3 Chemins

```python
DATA_DIR  = <package_share>/config/data_pairs/   # fichiers JSON QA
INDEX_DIR = <package_share>/config/index/         # fichiers .faiss + .json
```

### 4.4 Paramètres ROS2 (actuels — `qbo_aiml.yaml`)

```yaml
/qbo_aiml:
  ros__parameters:
    threshold_listen:     0.70
    threshold_diagnostic: 0.88
    threshold_dialog:     0.75
    enable_llm_rewrite:   false
    embed_model:          "intfloat/e5-small-v2"
```

---

## 5. Format des données QA (`data_pairs/*.json`)

Chaque fichier est une liste d'entrées :

```json
[
  {
    "question_variants": [
      "comment vas-tu",
      "tu vas bien",
      "ça va ?"
    ],
    "answer": [
      "Je vais très bien, merci !",
      "Tout roule de mon côté."
    ],
    "meta": {
      "intent_kind": "conversation",
      "domain":      "social"
    }
  }
]
```

| Champ               | Obligatoire | Description                                      |
|---------------------|-------------|--------------------------------------------------|
| `question_variants` | oui         | Phrases d'entraînement (dépliées dans l'index)   |
| `answer`            | oui         | Template(s) de réponse — liste ou string         |
| `meta.intent_kind`  | oui         | `listen` \| `conversation` \| `diagnostic`       |
| `meta.domain`       | non         | Domaine sémantique                               |
| `meta.component`    | non         | Composant hardware (diagnostics)                 |
| `intent`            | non         | Présence de ce champ → seuil `listen` (0.70)     |

**Variables dynamiques dans `answer` :**

```json
"answer": "Je mets le nez en {color_name}."
```
La valeur `{color_name}` est injectée depuis `context_json.params.color_name`
transmis par le SBE lors de l'appel.

---

## 6. Gestion des index FAISS

### Fichiers produits

```
config/index/
├── index_2026_08_23_10_30.faiss   ← vecteurs de toutes les questions
├── index_2026_08_23_10_30.json    ← liste [{question, entry}] (même ordre)
├── diag_2026_08_23_10_30.faiss    ← vecteurs des diagnostics seulement
└── diag_2026_08_23_10_30.json
```

`load_latest_index()` trie les fichiers par nom (horodatage ISO) et charge
le plus récent. L'index précédent reste sur disque (pas de suppression).

### Reconstruction

```bash
# Via ROS2 (node démarré)
ros2 service call /aiml/vectorize_index std_srvs/srv/Trigger

# Sans ROS2 (standalone)
python test_engine.py --batch --calibrate
```

---

## 7. Outil de test standalone (`test_engine.py`)

Fonctionne sans ROS2 — lit les fichiers QA et teste directement le QALoader.

```bash
# Régression complète
python test_engine.py --batch

# Calibration des seuils
python test_engine.py --calibrate

# Filtré par domaine
python test_engine.py --batch --domain battery

# Sauvegarde rapport JSON
python test_engine.py --batch --calibrate --save rapport.json
```

**Statuts batch :**

| Statut       | Signification                                              |
|--------------|------------------------------------------------------------|
| `PASS`       | Top-1 = entrée source ET score ≥ seuil                    |
| `FAIL_SCORE` | Top-1 correct MAIS score < seuil (seuil trop strict)      |
| `FAIL_WRONG` | Top-1 ≠ entrée source (ambiguïté à corriger dans les QA)  |

---

## 8. Intégration domotique — Proposition

### Problème actuel

Le nœud AIML répond avec du texte libre. Le SBE doit déterminer lui-même
si la réponse correspond à une commande domotique. Il n'y a pas de mécanisme
de détection ni de routage vers `/home_intent/parse`.

### Solution proposée : `meta.intent_kind = "home_control"`

#### Étape 1 — Marquer les entrées domotique dans les QA

```json
{
  "question_variants": [
    "allume le salon",
    "allume la lumière du salon",
    "mets la lumière dans le salon"
  ],
  "answer": "Je vais allumer le salon.",
  "meta": {
    "intent_kind": "home_control",
    "domain":      "light"
  }
}
```

#### Étape 2 — Détecter le `intent_kind` dans `aiml.py`

Modifier `generate_text_callback` (mode conversation) pour préfixer
`response_text` quand l'entrée est de type `home_control` :

```python
intent_kind = best_item.get("meta", {}).get("intent_kind", "")

if intent_kind == "home_control":
    # Préfixe structuré parseable par le SBE
    response.response_text = f"[HOME_INTENT]{sentence}|{base_answer}"
else:
    response.response_text = base_answer
```

Format retourné : `[HOME_INTENT]allume le salon|Je vais allumer le salon.`

#### Étape 3 — Parsing côté SBE

```python
text = resp.response_text
if text.startswith("[HOME_INTENT]"):
    payload = text[len("[HOME_INTENT]"):]
    command, spoken = payload.split("|", 1)
    # Vocaliser `spoken`
    tts_publish(spoken)
    # Envoyer `command` à home_intent
    home_intent_call(command, resolve=True, execute=True)
else:
    tts_publish(text)
```

#### Étape 4 — Seuil dédié pour `home_control`

Ajouter dans `THRESHOLDS` :

```python
THRESHOLDS = {
    "listen":       0.70,
    "home_control": 0.72,   # légèrement plus strict que listen
    "diagnostic":   0.88,
    "dialog":       0.75,
}
```

Et dans `generate_text_callback` :

```python
if "intent" in best_item or intent_kind == "home_control":
    threshold = self.THRESHOLDS.get(intent_kind, self.THRESHOLDS["listen"])
else:
    threshold = self.THRESHOLDS["dialog"]
```

### Avantages de cette approche

| Critère                  | Valeur                                                   |
|--------------------------|----------------------------------------------------------|
| Modification minimale    | Aucun changement de `.srv`, aucune nouvelle interface    |
| Réversible               | Retrait du préfixe si le SBE ne le détecte pas          |
| Traçable                 | Chaque entrée QA déclare explicitement son intent_kind   |
| Testable sans ROS2       | `test_engine.py` voit `meta.intent_kind`                 |
| Compatible `home_intent` | Le texte brut est transmis tel quel à `/home_intent/parse`|

### Alternatives

| Option                        | Avantage                        | Inconvénient                    |
|-------------------------------|---------------------------------|---------------------------------|
| Nouveau type `"home_control"` | Interface propre                | Modification du `GenerateText.srv` |
| Index FAISS dédié domotique   | Seuil indépendant               | 3ème index à maintenir          |
| Regex pré-AIML côté SBE       | Zéro modification AIML          | Duplication logique NLP         |
| **Préfixe `[HOME_INTENT]`**   | **Aucune interface à modifier** | **SBE doit parser le préfixe** |

---

## 9. Architecture complète avec domotique

```text
                    Entrée vocale (texte)
                           │
                           ▼
                    /aiml/generate_text
                           │
              ┌────────────┴────────────┐
              │                         │
     intent_kind=home_control    intent_kind=conversation
              │                         │
    [HOME_INTENT]cmd|spoken        spoken seul
              │                         │
              ▼                         ▼
           SBE                       TTS
              │
       ┌──────┴──────┐
       │             │
     TTS      /home_intent/parse
    (spoken)   {text=cmd, resolve=true, execute=true}
                      │
               HomeIntentNode
                      │
               /ha_bridge/call_service
                      │
               Home Assistant (KNX…)
```
