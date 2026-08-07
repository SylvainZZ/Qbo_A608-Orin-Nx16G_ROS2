# Snapshot of a single Home Assistant entity state
builtin_interfaces/Time stamp
string entity_id        # e.g.  light.salon
string domain           # e.g.  light
string friendly_name    # human-readable label from attributes
string state            # on / off / 22.5 / unavailable …
string attributes       # full JSON dict of extra HA attributes
string area_id          # room/zone id, empty when unassigned
