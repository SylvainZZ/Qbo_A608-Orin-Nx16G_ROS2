#!/usr/bin/env python3
"""
test_engine.py — Moteur de test standalone pour le RAG AIML
════════════════════════════════════════════════════════════

Principe 1 — Batch automatique (régression / couverture)
    Lit tous les fichiers JSON de data_pairs/, soumet CHAQUE question_variant
    au QALoader et vérifie que le top-1 retourné est bien l'entrée source,
    avec un score au-dessus du seuil de son domaine.

Principe 4 — Calibration des seuils
    Pour chaque domaine (listen / dialog / diagnostic), balaie les seuils de
    0.50 à 0.95 par pas de 0.05 et calcule TP / FP / FN → précision / rappel.
    Aide à choisir le meilleur seuil sans sacrifier la couverture.

Usage :
    python test_engine.py --batch
    python test_engine.py --calibrate
    python test_engine.py --batch --calibrate
    python test_engine.py --batch --domain battery
    python test_engine.py --batch --calibrate --save report.json
    python test_engine.py --batch --verbose
"""

import os
import sys
import json
import glob
import argparse
import hashlib
import time
from pathlib import Path
from typing import Optional

# ─────────────────────────────────────────────────────────────────────────────
# PATH SETUP — standalone, sans ROS2
# ─────────────────────────────────────────────────────────────────────────────
_HERE = Path(__file__).resolve()

# Détection automatique : workspace ROS2 (src/) vs package installé (site-packages)
def _find_package_dir():
    """Trouve le répertoire qbo_driver dans src/ ou fallback sur parents."""
    current = _HERE

    # Cas 1 : chercher un workspace ROS2 (remonter jusqu'à trouver src/qbo_driver)
    for parent in [_HERE] + list(_HERE.parents):
        ws_src_pkg = parent / "src" / "qbo_driver"
        if ws_src_pkg.is_dir() and (ws_src_pkg / "config").is_dir():
            return ws_src_pkg

        # Si on trouve qbo_ws/src/qbo_driver
        if parent.name == "qbo_ws":
            candidate = parent / "src" / "qbo_driver"
            if candidate.is_dir():
                return candidate

    # Cas 2 : exécution depuis le package source (test_engine.py est dans qbo_aiml/)
    # _HERE.parents[0] = qbo_aiml/
    # _HERE.parents[1] = qbo_driver/
    pkg_dir = _HERE.parents[1]
    if (pkg_dir / "config").is_dir():
        return pkg_dir

    # Cas 3 : fallback sur l'ancien comportement
    return _HERE.parents[2]

_PKG_DIR = _find_package_dir()

sys.path.insert(0, str(_PKG_DIR.parent))  # ajoute le parent pour "from qbo_driver..."

DATA_DIR         = str(_PKG_DIR / "config" / "data_pairs")
INDEX_DIR        = str(_PKG_DIR / "config" / "index")
EMBED_MODEL_NAME = "intfloat/e5-small-v2"

# ─────────────────────────────────────────────────────────────────────────────
# SEUILS PAR DOMAINE (miroir de aiml.py)
# ─────────────────────────────────────────────────────────────────────────────
THRESHOLDS = {
    "listen":     0.70,
    "diagnostic": 0.88,
    "dialog":     0.75,
}

# Mapping intent_kind → catégorie de seuil
INTENT_KIND_TO_CATEGORY = {
    "get":          "listen",
    "command":      "listen",
    "conversation": "dialog",
    "diagnostic":   "diagnostic",
}

CALIBRATION_RANGE = [round(t, 2) for t in [x / 100 for x in range(50, 96, 5)]]

# ─────────────────────────────────────────────────────────────────────────────
# LOGGER SILENCIEUX (supprime les logs internes de QALoader)
# ─────────────────────────────────────────────────────────────────────────────
class _SilentLogger:
    """Absorbe les logs internes de QALoader pendant les tests."""
    def info(self, msg):  pass
    def warn(self, msg):  pass
    def error(self, msg): print(f"\033[91m[QALoader ERROR] {msg}\033[0m")


class _VerboseLogger:
    """Affiche tous les logs QALoader (mode debug)."""
    def info(self, msg):  print(f"\033[90m  · {msg}\033[0m")
    def warn(self, msg):  print(f"\033[93m  ! {msg}\033[0m")
    def error(self, msg): print(f"\033[91m  ✗ {msg}\033[0m")


# ─────────────────────────────────────────────────────────────────────────────
# COULEURS TERMINAL
# ─────────────────────────────────────────────────────────────────────────────
class C:
    BOLD    = "\033[1m"
    GREEN   = "\033[92m"
    RED     = "\033[91m"
    YELLOW  = "\033[93m"
    CYAN    = "\033[96m"
    MAGENTA = "\033[95m"
    GREY    = "\033[90m"
    RESET   = "\033[0m"

    @staticmethod
    def ok(s):       return f"{C.GREEN}{s}{C.RESET}"
    @staticmethod
    def fail(s):     return f"{C.RED}{s}{C.RESET}"
    @staticmethod
    def warn(s):     return f"{C.YELLOW}{s}{C.RESET}"
    @staticmethod
    def title(s):    return f"{C.BOLD}{C.CYAN}{s}{C.RESET}"
    @staticmethod
    def sub(s):      return f"{C.MAGENTA}{s}{C.RESET}"
    @staticmethod
    def grey(s):     return f"{C.GREY}{s}{C.RESET}"


# ─────────────────────────────────────────────────────────────────────────────
# UTILITAIRES
# ─────────────────────────────────────────────────────────────────────────────
def _entry_id(entry: dict) -> str:
    """Identifiant stable d'une entrée QA basé sur ses question_variants."""
    variants = entry.get("question_variants", [entry.get("question", "")])
    key = json.dumps(sorted(variants), ensure_ascii=False)
    return hashlib.md5(key.encode()).hexdigest()[:8]


def _category(entry: dict) -> str:
    """Retourne la catégorie de seuil applicable à une entrée."""
    intent_kind = entry.get("meta", {}).get("intent_kind", "conversation")
    return INTENT_KIND_TO_CATEGORY.get(intent_kind, "dialog")


def _threshold(entry: dict) -> float:
    return THRESHOLDS[_category(entry)]


def _meta_summary(entry: dict) -> str:
    m = entry.get("meta", {})
    return f"{m.get('domain','?')} / {m.get('intent_kind','?')}"


def load_qa_files(data_dir: str, domain_filter: Optional[str] = None) -> list:
    """Charge toutes les entrées QA depuis les fichiers JSON du dossier data_pairs."""
    entries = []
    for fpath in sorted(glob.glob(os.path.join(data_dir, "*.json"))):
        fname = os.path.basename(fpath)
        with open(fpath, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, list):
            print(C.warn(f"  ⚠ Ignoré (format invalide) : {fname}"))
            continue
        for e in data:
            if domain_filter:
                m = e.get("meta", {})
                meta_domain  = m.get("domain", "")
                intent_kind  = m.get("intent_kind", "")
                mapped_cat   = INTENT_KIND_TO_CATEGORY.get(intent_kind, "dialog")
                if domain_filter not in (meta_domain, intent_kind, mapped_cat):
                    continue
            e["_source_file"] = fname
            entries.append(e)
    return entries


# ─────────────────────────────────────────────────────────────────────────────
# PRINCIPE 1 — BATCH TESTER
# ─────────────────────────────────────────────────────────────────────────────
class BatchTester:
    """
    Teste chaque question_variant de chaque entrée QA.

    Résultats possibles pour chaque variant :
      PASS       — top-1 = entrée source ET score ≥ seuil du domaine
      FAIL_SCORE — top-1 = entrée source MAIS score < seuil  (seuil trop haut ?)
      FAIL_WRONG — top-1 ≠ entrée source  (ambiguïté RAG)
    """

    def __init__(self, qa_loader, verbose: bool = False):
        self.loader  = qa_loader
        self.verbose = verbose
        self.results = []   # une entrée par variant testé

    def run(self, entries: list) -> dict:
        print()
        print(C.title(f"═══ BATCH TEST — {len(entries)} entrées QA ═══"))
        print()

        t0 = time.time()

        for entry in entries:
            eid       = _entry_id(entry)
            variants  = entry.get("question_variants", [entry.get("question", "")])
            threshold = _threshold(entry)
            category  = _category(entry)
            answer_preview = str(entry.get("answer", ""))[:60].replace("\n", " ")

            for variant in variants:
                candidates = self.loader.search_topk(variant, k=5)

                if not candidates:
                    self._record("NO_INDEX", eid, variant, category,
                                 threshold, 0.0, None, entry)
                    continue

                top = candidates[0]
                top_score = top["score"]
                top_id    = _entry_id(top["item"])

                if top_id == eid:
                    if top_score >= threshold:
                        status = "PASS"
                    else:
                        status = "FAIL_SCORE"
                else:
                    status = "FAIL_WRONG"

                self._record(status, eid, variant, category, threshold,
                             top_score, top["item"], entry)

                if self.verbose or status != "PASS":
                    self._print_row(status, variant, top_score, threshold,
                                    category, top["item"], entry)

        elapsed = time.time() - t0
        summary = self._summarize(elapsed)
        return summary

    def _record(self, status, eid, variant, category, threshold,
                score, top_item, source_entry):
        self.results.append({
            "status":        status,
            "entry_id":      eid,
            "variant":       variant,
            "category":      category,
            "threshold":     threshold,
            "score":         round(score, 4),
            "source_file":   source_entry.get("_source_file", "?"),
            "source_answer": str(source_entry.get("answer", ""))[:80],
            "top_answer":    str(top_item.get("answer", ""))[:80] if top_item else "",
        })

    def _print_row(self, status, variant, score, threshold, category,
                   top_item, source_entry):
        indicator = {
            "PASS":       C.ok("  ✔ PASS      "),
            "FAIL_SCORE": C.warn("  ⚠ FAIL_SCORE"),
            "FAIL_WRONG": C.fail("  ✗ FAIL_WRONG"),
            "NO_INDEX":   C.fail("  ✗ NO_INDEX  "),
        }.get(status, status)

        print(f"{indicator}  {C.grey(f'[{category:<11}]')}  "
              f"score={C.cyan_score(score, threshold):>5}  "
              f"seuil={threshold:.2f}  │  « {variant[:60]} »")

        if status == "FAIL_WRONG" and top_item:
            top_variants = top_item.get("question_variants", [])
            top_preview  = top_variants[0][:55] if top_variants else "?"
            print(f"{'':>15}  {C.grey('→ retourné :')} « {top_preview} »  "
                  f"{C.grey(_meta_summary(top_item))}")

    def _summarize(self, elapsed: float) -> dict:
        totals   = len(self.results)
        by_status = {}
        for r in self.results:
            by_status.setdefault(r["status"], []).append(r)

        n_pass  = len(by_status.get("PASS", []))
        n_fs    = len(by_status.get("FAIL_SCORE", []))
        n_fw    = len(by_status.get("FAIL_WRONG", []))
        n_ni    = len(by_status.get("NO_INDEX", []))

        pct = (n_pass / totals * 100) if totals else 0

        print()
        print(C.title("── Résumé Batch ──────────────────────────────────────"))
        print(f"  Variants testés  : {C.BOLD}{totals}{C.RESET}")
        print(f"  {C.ok('PASS')}           : {n_pass}  ({pct:.1f}%)")
        print(f"  {C.warn('FAIL_SCORE')}     : {n_fs}   (seuil trop strict ?)")
        print(f"  {C.fail('FAIL_WRONG')}     : {n_fw}   (ambiguïté RAG)")
        if n_ni:
            print(f"  {C.fail('NO_INDEX')}      : {n_ni}")
        print(f"  Durée            : {elapsed:.1f}s")

        # Détail par domaine
        by_cat = {}
        for r in self.results:
            cat = r["category"]
            by_cat.setdefault(cat, {"PASS": 0, "FAIL_SCORE": 0, "FAIL_WRONG": 0})
            by_cat[cat][r["status"]] = by_cat[cat].get(r["status"], 0) + 1

        print()
        print(C.sub("  Par domaine :"))
        for cat, counts in sorted(by_cat.items()):
            total_cat = sum(counts.values())
            p = counts.get("PASS", 0)
            pct_cat = (p / total_cat * 100) if total_cat else 0
            bar = self._bar(p, total_cat)
            print(f"  {cat:<12}  {bar}  {p}/{total_cat}  ({pct_cat:.0f}%)")

        print()

        # Top 10 FAIL_WRONG
        fails = by_status.get("FAIL_WRONG", [])[:10]
        if fails:
            print(C.sub("  Top FAIL_WRONG (ambiguïtés à corriger) :"))
            for r in fails:
                print(f"  {C.grey('·')} [{r['source_file']:<25}]  "
                      f"score={r['score']:.3f}  « {r['variant'][:55]} »")
                if r["top_answer"]:
                    print(f"    {'':>4}{C.grey('→ retourné :')} « {r['top_answer'][:60]} »")
            print()

        return {
            "total":       totals,
            "pass":        n_pass,
            "fail_score":  n_fs,
            "fail_wrong":  n_fw,
            "no_index":    n_ni,
            "coverage_pct": round(pct, 2),
            "by_category": by_cat,
            "details":     self.results,
        }

    @staticmethod
    def _bar(n, total, width=20):
        if total == 0:
            return "─" * width
        filled = int(n / total * width)
        return C.ok("█" * filled) + C.grey("░" * (width - filled))


def cyan_score(score, threshold):
    if score >= threshold:
        return C.ok(f"{score:.3f}")
    elif score >= threshold - 0.05:
        return C.warn(f"{score:.3f}")
    else:
        return C.fail(f"{score:.3f}")

# Patch méthode statique
C.cyan_score = staticmethod(cyan_score)


# ─────────────────────────────────────────────────────────────────────────────
# PRINCIPE 4 — CALIBRATION DES SEUILS
# ─────────────────────────────────────────────────────────────────────────────
class ThresholdCalibrator:
    """
    Pour chaque domaine, balaie les seuils de 0.50 à 0.95 (pas de 0.05).

    À chaque seuil :
      TP = score ≥ seuil  ET  entrée correcte
      FP = score ≥ seuil  ET  entrée incorrecte
      FN = score <  seuil ET  entrée correcte (résultat raté)
      TN = score <  seuil ET  entrée incorrecte (rejeté à juste titre)

      Précision  = TP / (TP + FP)
      Rappel     = TP / (TP + FN)
      F1         = 2 * P * R / (P + R)
    """

    def __init__(self, qa_loader):
        self.loader = qa_loader

    def run(self, entries: list) -> dict:
        print()
        print(C.title("═══ CALIBRATION SEUILS ═══"))
        print()

        # 1. Collecter tous les scores bruts une seule fois
        raw = []   # { entry_id, category, score_correct (meilleur score du bon item), score_top }
        print(f"  Collecte des scores pour {len(entries)} entrées...", end="", flush=True)
        t0 = time.time()

        for entry in entries:
            eid      = _entry_id(entry)
            category = _category(entry)
            variants = entry.get("question_variants", [entry.get("question", "")])

            for variant in variants:
                candidates = self.loader.search_topk(variant, k=5)
                if not candidates:
                    continue

                top      = candidates[0]
                top_id   = _entry_id(top["item"])
                top_score = top["score"]

                # Chercher le score de l'entrée correcte dans le top-k
                correct_score = next(
                    (c["score"] for c in candidates if _entry_id(c["item"]) == eid),
                    0.0
                )

                raw.append({
                    "category":      category,
                    "top_id_correct": top_id == eid,
                    "top_score":     top_score,
                    "correct_score": correct_score,
                })

        elapsed = time.time() - t0
        print(f"  {len(raw)} variants traités en {elapsed:.1f}s")

        # 2. Calculer métriques par domaine et par seuil
        categories = sorted(set(r["category"] for r in raw))
        report = {}

        for cat in categories:
            subset = [r for r in raw if r["category"] == cat]
            if not subset:
                continue

            current_threshold = THRESHOLDS[cat]
            rows = []

            for thresh in CALIBRATION_RANGE:
                tp = fp = fn = tn = 0
                for r in subset:
                    accepted = r["top_score"] >= thresh
                    correct  = r["top_id_correct"]
                    if accepted and correct:     tp += 1
                    elif accepted and not correct: fp += 1
                    elif not accepted and correct: fn += 1
                    else:                         tn += 1

                precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
                recall    = tp / (tp + fn) if (tp + fn) > 0 else 0.0
                f1        = (2 * precision * recall / (precision + recall)
                             if (precision + recall) > 0 else 0.0)
                coverage  = tp / len(subset) if subset else 0.0

                rows.append({
                    "threshold": thresh,
                    "tp": tp, "fp": fp, "fn": fn, "tn": tn,
                    "precision": round(precision, 3),
                    "recall":    round(recall, 3),
                    "f1":        round(f1, 3),
                    "coverage":  round(coverage, 3),
                })

            report[cat] = {
                "current_threshold": current_threshold,
                "n_variants": len(subset),
                "rows": rows,
            }

            self._print_table(cat, rows, current_threshold, len(subset))

        return report

    def _print_table(self, category: str, rows: list,
                     current: float, n: int):
        print(C.sub(f"\n  ┌── Domaine : {category.upper()} "
                    f"({n} variants) ──────────────────────────────────────────────┐"))
        print(C.grey(
            f"  │  {'Seuil':>6}  {'TP':>5}  {'FP':>5}  {'FN':>5}  "
            f"{'Précision':>10}  {'Rappel':>8}  {'F1':>6}  {'Couverture':>10}  │"
        ))
        print(C.grey("  │" + "─" * 73 + "│"))

        best_f1  = max(r["f1"] for r in rows)

        for r in rows:
            is_current = abs(r["threshold"] - current) < 0.001
            is_best_f1 = abs(r["f1"] - best_f1) < 0.001

            marker = ""
            if is_current and is_best_f1:
                marker = C.ok(" ◀ actuel + optimal F1")
            elif is_current:
                marker = C.warn(" ◀ actuel")
            elif is_best_f1:
                marker = C.ok(" ★ optimal F1")

            # Mise en couleur de la ligne selon F1
            if r["f1"] == best_f1:
                col = C.GREEN
            elif is_current:
                col = C.YELLOW
            else:
                col = C.GREY

            print(
                f"  │{col}  {r['threshold']:.2f}   "
                f"{r['tp']:>5}  {r['fp']:>5}  {r['fn']:>5}  "
                f"{r['precision']:>10.3f}  {r['recall']:>8.3f}  "
                f"{r['f1']:>6.3f}  {r['coverage']:>10.3f}"
                f"{C.RESET}{marker}"
            )

        print(C.grey("  └" + "─" * 73 + "┘"))


# ─────────────────────────────────────────────────────────────────────────────
# POINT D'ENTRÉE
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Moteur de test standalone AIML — Principes 1 et 4"
    )
    parser.add_argument("--batch",      action="store_true",
                        help="Principe 1 : test de régression/couverture")
    parser.add_argument("--calibrate",  action="store_true",
                        help="Principe 4 : calibration des seuils par domaine")
    parser.add_argument("--domain",     type=str, default=None,
                        help="Filtre par domaine : battery, dialog, diagnostic…")
    parser.add_argument("--save",       type=str, default=None,
                        metavar="FILE.json",
                        help="Sauvegarde le rapport JSON dans FILE.json")
    parser.add_argument("--verbose",    action="store_true",
                        help="Affiche toutes les lignes (y compris PASS)")
    parser.add_argument("--data-dir",   type=str, default=DATA_DIR)
    parser.add_argument("--index-dir",  type=str, default=INDEX_DIR)
    args = parser.parse_args()

    if not args.batch and not args.calibrate:
        parser.print_help()
        sys.exit(0)

    # ── Import QALoader (ROS2 non requis) ────────────────────────────────────
    try:
        from qbo_driver.qbo_aiml.qa_loader import QALoader
    except ImportError as e:
        print(C.fail(f"❌ Impossible d'importer QALoader : {e}"))
        print(C.grey(f"   Vérifie que {_PKG_DIR} est accessible."))
        sys.exit(1)

    # ── Chargement du modèle ─────────────────────────────────────────────────
    logger = _VerboseLogger() if args.verbose else _SilentLogger()

    print(C.title("\n  AIML Test Engine"))
    print(C.grey(f"  data_dir  : {args.data_dir}"))
    print(C.grey(f"  index_dir : {args.index_dir}"))
    print(C.grey(f"  model     : {EMBED_MODEL_NAME}"))
    if args.domain:
        print(C.grey(f"  domaine   : {args.domain}"))
    print()

    print("  Chargement du modèle d'embedding...", end=" ", flush=True)
    t0 = time.time()
    loader = QALoader(
        model_name=EMBED_MODEL_NAME,
        logger=logger,
        data_dir=args.data_dir,
        index_dir=args.index_dir,
    )
    loader.load_latest_index(prefix="index")
    print(f"prêt ({time.time()-t0:.1f}s)")

    if not loader.index:
        print(C.fail("\n  ❌ Aucun index FAISS trouvé."))
        print(C.warn("  → Lance d'abord : ros2 service call /aiml/vectorize_index"))
        sys.exit(1)

    # ── Chargement des données QA ────────────────────────────────────────────
    print(f"  Chargement des fichiers QA depuis {args.data_dir}...", end=" ", flush=True)
    entries = load_qa_files(args.data_dir, domain_filter=args.domain)
    print(f"{len(entries)} entrées")

    if not entries:
        print(C.fail("  ❌ Aucune entrée QA trouvée."))
        sys.exit(1)

    # ── Exécution ────────────────────────────────────────────────────────────
    report = {}

    if args.batch:
        tester = BatchTester(loader, verbose=args.verbose)
        report["batch"] = tester.run(entries)

    if args.calibrate:
        calibrator = ThresholdCalibrator(loader)
        report["calibration"] = calibrator.run(entries)

    # ── Sauvegarde rapport ───────────────────────────────────────────────────
    if args.save:
        save_path = Path(args.save)
        # On ne sérialise pas les détails variants dans la calibration (trop volumineux)
        with open(save_path, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print(C.ok(f"\n  💾 Rapport sauvegardé : {save_path.resolve()}"))

    print()


if __name__ == "__main__":
    main()
