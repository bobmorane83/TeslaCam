#!/bin/bash
# =============================================================================
# Tesla CAN Dashboard - Lanceur
# Se connecte au WiFi "bridge", installe les dépendances et lance le dashboard
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

echo "==========================================="
echo "  Tesla CAN Dashboard - Lanceur"
echo "==========================================="

# Créer le virtualenv si nécessaire
if [ ! -d "$VENV_DIR" ]; then
    echo "[SETUP] Création de l'environnement virtuel..."
    python3 -m venv "$VENV_DIR"
fi

# Activer le virtualenv
source "$VENV_DIR/bin/activate"

# Installer les dépendances
echo "[SETUP] Vérification des dépendances..."
pip install -q -r "$SCRIPT_DIR/requirements.txt" 2>/dev/null

# Lancer le dashboard
echo "[START] Nettoyage des anciens processus..."
pkill -9 -f "python.*app.py" 2>/dev/null
sleep 1
# Libérer le port UDP 5555 si un socket orphelin traîne
lsof -ti :5555 | xargs kill -9 2>/dev/null
sleep 1
echo "[START] Lancement du dashboard..."
echo ""
python3 "$SCRIPT_DIR/app.py"
