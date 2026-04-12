#!/usr/bin/env bash
# =============================================================================
# QBO Bringup Manager — CLI Helper
# =============================================================================
# Script utilitaire pour faciliter la gestion des profils via ligne de commande
#
# Usage:
#   ./qbo_profile.sh start MINIMAL
#   ./qbo_profile.sh stop VISION
#   ./qbo_profile.sh status FULL
#   ./qbo_profile.sh restart MINIMAL
#   ./qbo_profile.sh list
#
# =============================================================================

set -e

# Couleurs pour affichage
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

SERVICE_NAME="/qbo_bringup/manage_profile"
SERVICE_TYPE="qbo_msgs/srv/ManageProfile"

# =============================================================================
# FONCTIONS UTILITAIRES
# =============================================================================

print_header() {
    echo -e "${CYAN}======================================================================${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}======================================================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

check_service_available() {
    if ! ros2 service list 2>/dev/null | grep -q "$SERVICE_NAME"; then
        print_error "Service $SERVICE_NAME non disponible"
        print_warning "Démarrez d'abord : ros2 run qbo_bringup qbo_bringup_manager"
        exit 1
    fi
}

# =============================================================================
# COMMANDES
# =============================================================================

cmd_start() {
    local profile=$1
    if [ -z "$profile" ]; then
        print_error "Usage: $0 start <PROFILE>"
        exit 1
    fi

    print_header "Démarrage du profil $profile"
    check_service_available

    ros2 service call "$SERVICE_NAME" "$SERVICE_TYPE" \
        "{profile_name: '$profile', action: 'START'}"
}

cmd_stop() {
    local profile=$1
    if [ -z "$profile" ]; then
        print_error "Usage: $0 stop <PROFILE>"
        exit 1
    fi

    print_header "Arrêt du profil $profile"
    check_service_available

    ros2 service call "$SERVICE_NAME" "$SERVICE_TYPE" \
        "{profile_name: '$profile', action: 'STOP'}"
}

cmd_restart() {
    local profile=$1
    if [ -z "$profile" ]; then
        print_error "Usage: $0 restart <PROFILE>"
        exit 1
    fi

    print_header "Redémarrage du profil $profile"
    check_service_available

    ros2 service call "$SERVICE_NAME" "$SERVICE_TYPE" \
        "{profile_name: '$profile', action: 'RESTART'}"
}

cmd_status() {
    local profile=$1
    if [ -z "$profile" ]; then
        print_error "Usage: $0 status <PROFILE>"
        exit 1
    fi

    print_header "État du profil $profile"
    check_service_available

    ros2 service call "$SERVICE_NAME" "$SERVICE_TYPE" \
        "{profile_name: '$profile', action: 'STATUS'}"
}

cmd_list() {
    print_header "Profils disponibles"
    echo ""
    echo -e "${GREEN}MINIMAL${NC}     - Nœuds essentiels (base + audio)"
    echo -e "${GREEN}VISION${NC}      - MINIMAL + détection visuelle"
    echo -e "${GREEN}NAVIGATION${NC}  - MINIMAL + navigation autonome"
    echo -e "${GREEN}FULL${NC}        - Tous les nœuds actifs"
    echo ""

    if ros2 service list 2>/dev/null | grep -q "$SERVICE_NAME"; then
        print_success "Service bringup_manager actif"
        echo ""
        echo "Pour voir les profils actifs :"
        echo "  ros2 service call $SERVICE_NAME $SERVICE_TYPE \\"
        echo "    \"{profile_name: 'MINIMAL', action: 'STATUS'}\""
    else
        print_warning "Service bringup_manager non démarré"
        echo "Démarrez-le avec : ros2 run qbo_bringup qbo_bringup_manager"
    fi
}

cmd_help() {
    print_header "QBO Profile Manager — CLI Helper"
    echo ""
    echo "Usage: $0 <command> [args]"
    echo ""
    echo "Commandes :"
    echo "  start <PROFILE>    Démarre un profil"
    echo "  stop <PROFILE>     Arrête un profil"
    echo "  restart <PROFILE>  Redémarre un profil"
    echo "  status <PROFILE>   Affiche l'état d'un profil"
    echo "  list               Liste les profils disponibles"
    echo "  help               Affiche cette aide"
    echo ""
    echo "Exemples :"
    echo "  $0 start MINIMAL"
    echo "  $0 stop VISION"
    echo "  $0 status FULL"
    echo ""
}

# =============================================================================
# MAIN
# =============================================================================

case "$1" in
    start)
        cmd_start "$2"
        ;;
    stop)
        cmd_stop "$2"
        ;;
    restart)
        cmd_restart "$2"
        ;;
    status)
        cmd_status "$2"
        ;;
    list)
        cmd_list
        ;;
    help|--help|-h|"")
        cmd_help
        ;;
    *)
        print_error "Commande inconnue : $1"
        echo ""
        cmd_help
        exit 1
        ;;
esac
