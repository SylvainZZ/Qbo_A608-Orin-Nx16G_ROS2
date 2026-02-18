import time
from collections import deque


'''🎯 Rôle de event_manager.py

Il s'occupe uniquement de :

    - détecter changement d'état
    - éviter répétitions
    - gérer cooldown
    - priorités (info / warning / error)
    - mettre en file d'attente
    - fournir un message à annoncer

Il ne publie PAS directement.
Il ne fait PAS de ROS.
Il ne parle PAS.

Il décide.
'''

class EventManager:

    def __init__(self):
        self.events_state = {}
        self.events_cooldown = {}
        self.event_queue = deque()

        # secondes minimum entre deux annonces identiques
        self.default_cooldown = 30

    def update_event(self, key: str, active: bool, message: str,
                     severity="warning", cooldown=None):
        """
        Met à jour un événement.

        - key : identifiant unique
        - active : True si problème présent
        - message : phrase à annoncer
        - severity : info / warning / error
        """

        now = time.time()
        cooldown = cooldown or self.default_cooldown

        previous_state = self.events_state.get(key, False)
        last_time = self.events_cooldown.get(key, 0)

        # 1️⃣ Si événement vient d'apparaître
        if active and not previous_state:
            self.event_queue.append({
                "message": message,
                "severity": severity
            })
            self.events_cooldown[key] = now

        # 2️⃣ Si toujours actif → vérifier cooldown
        elif active and previous_state:
            if (now - last_time) > cooldown:
                self.event_queue.append({
                    "message": message,
                    "severity": severity
                })
                self.events_cooldown[key] = now

        # 3️⃣ Si problème résolu
        elif not active and previous_state:
            self.event_queue.append({
                "message": f"{key.replace('_', ' ')} résolu.",
                "severity": "info"
            })

        self.events_state[key] = active

    def get_next_event(self):
        if self.event_queue:
            return self.event_queue.popleft()
        return None