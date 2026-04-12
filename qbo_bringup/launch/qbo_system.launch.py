#!/usr/bin/env python3
"""
QBO System Launch — Démarrage complet du système QBO Social avec Bringup Manager

Lance tous les nœuds essentiels dans l'ordre optimal :
  1. qbo_bringup_manager (gestion des profils)
  2. qbo_social (event_adapter, world_model, behavior_engine, action_executor)
  3. system_mode_manager (surveillance et orchestration)
  4. debug_behavior_state (optionnel, dashboard)

Usage:
    ros2 launch qbo_bringup qbo_system.launch.py

    # Avec dashboard
    ros2 launch qbo_bringup qbo_system.launch.py use_dashboard:=true

    # Sans profil auto-start
    ros2 launch qbo_bringup qbo_system.launch.py auto_start_profile:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # Arguments de lancement
    use_dashboard_arg = DeclareLaunchArgument(
        'use_dashboard',
        default_value='false',
        description='Lancer le dashboard debug_behavior_state'
    )

    auto_start_profile_arg = DeclareLaunchArgument(
        'auto_start_profile',
        default_value='MINIMAL',
        description='Profil à démarrer automatiquement (MINIMAL, VISION, FULL, ou "none")'
    )

    # Récupération des arguments
    use_dashboard = LaunchConfiguration('use_dashboard')
    auto_start_profile = LaunchConfiguration('auto_start_profile')

    return LaunchDescription([

        # =====================================================================
        # ARGUMENTS
        # =====================================================================
        use_dashboard_arg,
        auto_start_profile_arg,

        # =====================================================================
        # LOG STARTUP
        # =====================================================================
        LogInfo(msg="=" * 70),
        LogInfo(msg="QBO System Launch — Starting all components"),
        LogInfo(msg="=" * 70),

        # =====================================================================
        # 1. QBO BRINGUP MANAGER — Gestionnaire de profils
        # =====================================================================
        # Node(
        #     package='qbo_bringup',
        #     executable='qbo_bringup_manager',
        #     name='qbo_bringup_manager',
        #     output='screen',
        #     emulate_tty=True,
        # ),

        # LogInfo(msg="✓ qbo_bringup_manager started"),

        # =====================================================================
        # 2. QBO SOCIAL — Architecture comportementale
        # =====================================================================

        # Event Adapter (entrée du système)
        Node(
            package='qbo_social',
            executable='event_adapter',
            name='qbo_social_event_adapter',
            output='screen',  # Affiche les logs sur la console
            emulate_tty=True, # Permet d'avoir des logs colorés et formatés
        ),

        # World Model (état du monde)
        Node(
            package='qbo_social',
            executable='world_model',
            name='qbo_social_world_model',
            output='screen',
            emulate_tty=True,
        ),

        # Time Event Publisher (événements temporels)
        Node(
            package='qbo_social',
            executable='time_event_publisher',
            name='qbo_time_event_publisher',
            output='screen',
            emulate_tty=True,
        ),

        # Network Event Publisher (événements réseau)
        Node(
            package='qbo_social',
            executable='network_event_publisher',
            name='qbo_network_event_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'known_networks': ['Lune_Ext'],  # Personnalisable
                'test_host': '8.8.8.8',
            }]
        ),

        # Behavior Engine (prise de décision)
        Node(
            package='qbo_social',
            executable='behavior_engine',
            name='qbo_social_behavior_engine',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'greet_cooldown': 10.0,
                'stable_head_stop_delay': 5.0,
                'search_timeout': 30.0,
                'diag_cooldown': 5.0,
            }]
        ),

        # Action Executor (exécution des actions)
        Node(
            package='qbo_social',
            executable='action_executor',
            name='qbo_social_action_executor',
            output='screen',
            emulate_tty=True,
        ),

        LogInfo(msg="✓ qbo_social nodes started"),

        # =====================================================================
        # 3. SYSTEM MODE MANAGER — Surveillance système
        # =====================================================================
        Node(
            package='qbo_social',
            executable='system_mode_manager',
            name='qbo_system_mode_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'active_profile': auto_start_profile,
            }]
        ),

        LogInfo(msg="✓ system_mode_manager started"),

        # =====================================================================
        # 4. DEBUG DASHBOARD (optionnel)
        # =====================================================================
        Node(
            package='qbo_social',
            executable='debug_behavior_state',
            name='qbo_debug_dashboard',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(use_dashboard),
        ),

        LogInfo(
            msg="✓ debug_behavior_state started",
            condition=IfCondition(use_dashboard)
        ),

        # =====================================================================
        # FINAL LOG
        # =====================================================================
        LogInfo(msg="=" * 70),
        LogInfo(msg="QBO System fully operational"),
        LogInfo(msg="  Monitor: ros2 run qbo_social debug_behavior_state"),
        LogInfo(msg="  Manage profiles: ros2 service call /qbo_bringup/manage_profile"),
        LogInfo(msg="=" * 70),
    ])
