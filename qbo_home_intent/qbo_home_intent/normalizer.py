"""Text normalizer: lowercase, accent removal, contraction expansion."""

import re
import unicodedata

# French contractions expanded before accent removal
_CONTRACTIONS: list[tuple[str, str]] = [
    ("c'est", "cest"),
    ("l'", "le "),
    ("d'", "de "),
    ("j'", "je "),
    ("n'", "ne "),
    ("m'", "me "),
    ("qu'", "que "),
    ("s'", "se "),
]

_PUNCTUATION = re.compile(r"[^\w\s]")
_WHITESPACE = re.compile(r"\s+")


def normalize(text: str) -> str:
    """Return a normalized, lowercase, accent-free version of *text*."""
    text = text.lower().strip()
    for src, dst in _CONTRACTIONS:
        text = text.replace(src, dst)
    text = _strip_accents(text)
    text = _PUNCTUATION.sub(" ", text)
    text = _WHITESPACE.sub(" ", text).strip()
    return text


def _strip_accents(text: str) -> str:
    nfkd = unicodedata.normalize("NFKD", text)
    return "".join(c for c in nfkd if not unicodedata.combining(c))
