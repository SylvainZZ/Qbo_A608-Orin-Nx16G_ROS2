#!/usr/bin/env python3
"""
⚠️  DEPRECATED - Ce script a été fusionné avec debug_behavior_state.py

Pour le debug complet du behavior engine, utilisez maintenant:
    ros2 run qbo_social debug_behavior_state
    ros2 run qbo_social debug_behavior_state --verbose  (pour WorldState aussi)

Ce script redirige automatiquement vers debug_behavior_state.py
"""

import sys
import os

def main():
    print("\n" + "=" * 80)
    print("⚠️  NOTICE: decision_trace_monitor.py est maintenant intégré dans debug_behavior_state.py")
    print("=" * 80)
    print("\nPour une vue d'ensemble complète du behavior engine, utilisez:")
    print("  → ros2 run qbo_social debug_behavior_state")
    print("\nPour inclure les mises à jour WorldState:")
    print("  → ros2 run qbo_social debug_behavior_state --verbose")
    print("\n" + "=" * 80 + "\n")
    
    # Redirection automatique vers debug_behavior_state
    print("Redirection vers debug_behavior_state...\n")
    
    # Importer et lancer le nouveau script
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from debug_behavior_state import main as debug_main
    debug_main()


if __name__ == '__main__':
    main()

