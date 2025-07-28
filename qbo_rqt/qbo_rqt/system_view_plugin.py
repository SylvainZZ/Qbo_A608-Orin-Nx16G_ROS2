import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from diagnostic_msgs.msg import DiagnosticArray
import threading
import pygraphviz as pgv
import tempfile
import os
import subprocess

from textwrap import dedent
from pathlib import Path
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QScrollArea, QPushButton
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtCore import Qt, QTimer
from functools import partial
from collections import defaultdict

EXCLUDED_TOPICS = {
    '/rosout',
    '/parameter_events',
    '/diagnostics_toplevel_state',
}

EXCLUDED_NODE_PATTERNS = [
    'rqt_gui_py_node',
    'system_view_plugin_node'
]

DIAG_COLORS = {
    0: 'lightgreen',   # OK
    1: 'yellow',       # WARN
    2: 'red',          # ERROR
    3: 'gray',         # STALE / UNKNOWN
}

def is_excluded_node(name):
        return any(pat in name for pat in EXCLUDED_NODE_PATTERNS)

# def shorten_name(full_name):
#     if full_name.startswith('/qbo_arduqbo/'):
#         return full_name.replace('/qbo_arduqbo/', '')
#     elif full_name.startswith('/'):
#         return full_name[1:]
#     return full_name

# def extract_cluster_prefix(node_name):
#     parts = node_name.strip('/').split('/')
#     return parts[0] if len(parts) > 1 else None

def build_node_fullname(node_info):
    ns = node_info.node_namespace.rstrip('/')
    name = node_info.node_name
    return f"{ns}/{name}" if ns else f"/{name}"


class BackgroundNode(Node):
    def __init__(self):
        super().__init__('system_view_plugin_node')
        print("[SystemView] Node ROS 2 initialisé")


class SystemViewPlugin(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('SystemViewPlugin')

        print("[SystemView] Initialisation du plugin")

        self._widget = QWidget()
        self._widget.setObjectName('SystemViewWidget')
        self._widget.setWindowTitle('System View')

        self._layout = QVBoxLayout(self._widget)

        self._image_label = QLabel("Graphe en cours de génération...")
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setMinimumSize(800, 600)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._image_label)
        self._layout.addWidget(scroll)

        refresh_btn = QPushButton("Rafraîchir")
        refresh_btn.clicked.connect(self.update_graph)
        self._layout.addWidget(refresh_btn)

        view_btn = QPushButton("Ouvrir l'image originale")
        view_btn.clicked.connect(self.open_image)
        self._layout.addWidget(view_btn)

        self._last_graph_path = None

        self.auto_refresh_enabled = False
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.update_graph)

        self.auto_refresh_btn = QPushButton("Activer mise à jour auto")
        self.auto_refresh_btn.setCheckable(True)
        self.auto_refresh_btn.clicked.connect(self.toggle_auto_refresh)
        self._layout.addWidget(self.auto_refresh_btn)

        context.add_widget(self._widget)

        self._diagnostic_levels = {}  # Dict[str, str] → node name → level

        # Création du noeud + exécuteur
        print("[SystemView] Lancement de l'exécuteur dans un thread")
        self.executor = SingleThreadedExecutor()
        self.node = BackgroundNode()

        self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics_agg',
            self._diagnostic_callback,
            10
        )

        self.executor.add_node(self.node)
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # Rafraîchissement initial
        QTimer.singleShot(3000, self.update_graph)

    def open_image(self):
        if self._last_graph_path and os.path.exists(self._last_graph_path):
            subprocess.Popen(['xdg-open', self._last_graph_path])
        else:
            print("[SystemView] Aucun fichier image à ouvrir.")

    def toggle_auto_refresh(self):
        self.auto_refresh_enabled = not self.auto_refresh_enabled
        if self.auto_refresh_enabled:
            self.refresh_timer.start(5000)  # toutes les 5 secondes
            self.auto_refresh_btn.setText("Désactiver mise à jour auto")
        else:
            self.refresh_timer.stop()
            self.auto_refresh_btn.setText("Activer mise à jour auto")

    def _diagnostic_callback(self, msg):
        for status in msg.status:
            print(f"[Diag] {status.name} → level {status.level}")
            node_name = status.name.split('/')[0]  # ou traitement + précis
            level = status.level
            self._diagnostic_levels[node_name] = level

    def update_graph(self):
        print("[SystemView] Démarrage update_graph()")

        try:
            topic_list = self.node.get_topic_names_and_types()
            topic_list = [(t, ty) for (t, ty) in topic_list if t not in EXCLUDED_TOPICS]
            print(f"[SystemView] Topics trouvés : {len(topic_list)}")

            node_topic_map = {}

            for topic, types in topic_list:
                print(f"  ▶ Topic : {topic} - Types : {types}")
                publishers = self.node.get_publishers_info_by_topic(topic)
                subscribers = self.node.get_subscriptions_info_by_topic(topic)

                for pub in publishers:
                    pub_node = build_node_fullname(pub)
                    if is_excluded_node(pub_node):
                        continue
                    node_topic_map.setdefault(pub_node, {"pub": [], "sub": []})
                    node_topic_map[pub_node]["pub"].append(topic)

                for sub in subscribers:
                    sub_node = build_node_fullname(sub)
                    if is_excluded_node(sub_node):
                        continue
                    node_topic_map.setdefault(sub_node, {"pub": [], "sub": []})
                    node_topic_map[sub_node]["sub"].append(topic)

            print(f"[SystemView] Nombre de nœuds trouvés : {len(node_topic_map)}")
            print("[SystemView] Construction du graphe avec regroupement automatique par namespace")

            graph = pgv.AGraph(directed=True)

            # Étape 1 — Obtenir les noms et namespaces complets des nœuds
            node_names = self.node.get_node_names_and_namespaces()
            clusters = defaultdict(list)
            all_nodes_fullnames = set()

            for name, namespace in node_names:
                full_name = namespace.rstrip('/') + '/' + name if namespace != '/' else '/' + name
                if is_excluded_node(name):
                    continue
                clusters[namespace].append(full_name)
                all_nodes_fullnames.add(full_name)

            # Étape 2 — Créer les subgraphs par namespace
            for ns, nodes in clusters.items():
                if ns == '/' or ns.strip() == '':
                    continue  # Pas de subgraph pour racine

                cluster_name = f"cluster_{ns.strip('/') or 'root'}"
                print(f"[SystemView] Création du cluster : {cluster_name}")

                with graph.subgraph(name=cluster_name) as sub:
                    sub.graph_attr.update(label=ns, style='dashed', rankdir='TB')
                    for full_name in nodes:
                        node_label = full_name.split('/')[-1]
                        diag_key = full_name.strip('/')
                        level = self._diagnostic_levels.get(diag_key, 3)
                        color = DIAG_COLORS.get(level, 'gray')
                        print(f"[Color] {diag_key} → level {level} → {color}")
                        sub.add_node(full_name, shape="ellipse", style="filled", fontsize="6", fillcolor=color, label=node_label)

            # Étape 2 bis — Ajouter les nœuds hors cluster avec couleur
            for node in node_topic_map:
                if node in all_nodes_fullnames:
                    continue
                node_label = node.split('/')[-1]
                diag_key = node.strip('/')
                level = self._diagnostic_levels.get(diag_key, 3)
                color = DIAG_COLORS.get(level, 'gray')
                print(f"[Color] {node} (hors cluster) → level {level} → {color}")
                graph.add_node(node, shape="ellipse", style="filled", fontsize="6", fillcolor=color, label=node_label)

            # Étape 3 — Préparer le routage proxy pour les clusters
            cluster_headers = set()
            for ns, nodes in clusters.items():
                if ns == '/' or not nodes:
                    continue
                header_node = ns.rstrip('/')
                if header_node:
                    cluster_headers.add(header_node)

            cluster_representatives = {}
            for ns, nodes in clusters.items():
                if ns == '/' or not nodes:
                    continue
                header = ns.rstrip('/')
                representative = nodes[0]
                cluster_representatives[header] = representative

            # Étape 4 — Ajouter les topics et les connexions
            for node, info in node_topic_map.items():
                if node in cluster_headers:
                    proxy_node = cluster_representatives.get(node)
                    if not proxy_node:
                        continue
                    for topic in info["pub"]:
                        graph.add_node(topic, shape="plaintext", fontsize="6")
                        graph.add_edge(proxy_node, topic)
                    for topic in info["sub"]:
                        graph.add_node(topic, shape="plaintext", fontsize="6")
                        graph.add_edge(topic, proxy_node)
                    continue

                for topic in info["pub"]:
                    graph.add_node(topic, shape="plaintext", fontsize="6")
                    graph.add_edge(node, topic)
                for topic in info["sub"]:
                    graph.add_node(topic, shape="plaintext", fontsize="6")
                    graph.add_edge(topic, node)

            print(f"[SystemView] Graphe généré avec {len(graph.nodes())} nœuds.")

            with tempfile.NamedTemporaryFile(delete=False, suffix=".png") as f:
                graph.layout(prog='dot')
                graph.draw(f.name)
                pixmap = QPixmap(f.name)
                print(f"[DEBUG] Image size: {pixmap.size()}")
                if not pixmap.isNull():
                    self._image_label.setPixmap(pixmap)
                    self._image_label.setScaledContents(True)
                    self._image_label.repaint()
                    self._last_graph_path = f.name
                    print(f"[SystemView] Image affichée correctement.")
                else:
                    print(f"[SystemView][ERREUR] Pixmap vide ou invalide.")
                print(f"[SystemView] Image affichée : {f.name}")
                # os.unlink(f.name)

        except Exception as e:
            print(f"[SystemView][ERREUR] {e}")

    def shutdown_plugin(self):
        print("[SystemView] Fermeture du plugin")
        self.executor.shutdown()
        self.node.destroy_node()
