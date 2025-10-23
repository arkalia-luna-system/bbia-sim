#!/bin/bash

# 🛑 Script pour arrêter les processus gourmands
# Garde seulement Cursor, ChatGPT, Perplexity et VS Code

echo "🛑 Arrêt des processus gourmands..."
echo "=================================="

# Fonction pour vérifier si un processus doit être gardé
keep_process() {
    local cmdline="$1"
    local keep_apps=("Cursor" "ChatGPT" "Perplexity" "Visual Studio Code" "Code" "cursor" "chatgpt" "perplexity")
    
    for app in "${keep_apps[@]}"; do
        if [[ "$cmdline" == *"$app"* ]]; then
            return 0  # Garder le processus
        fi
    done
    return 1  # Arrêter le processus
}

# Identifier les processus gourmands (CPU > 10% ou RAM > 500MB)
echo "🔍 Recherche des processus gourmands..."

ps aux | awk 'NR>1 && ($3 > 10 || $6 > 512000) {
    cmdline = ""
    for (i=11; i<=NF; i++) cmdline = cmdline " " $i
    
    # Vérifier si le processus doit être gardé
    keep = 0
    if (cmdline ~ /Cursor|ChatGPT|Perplexity|Visual Studio Code|Code|cursor|chatgpt|perplexity/) {
        keep = 1
    }
    
    if (!keep) {
        printf "PID: %s, CPU: %s%%, RAM: %.1fMB, CMD: %s\n", $2, $3, $6/1024, cmdline
    }
}' | while read line; do
    if [[ -n "$line" ]]; then
        echo "⚠️  Processus gourmand trouvé: $line"
    fi
done

echo ""
echo "❓ Voulez-vous arrêter ces processus gourmands? (y/N)"
read -r response

if [[ "$response" =~ ^[Yy]$ ]]; then
    echo "🛑 Arrêt des processus gourmands..."
    
    # Arrêter les processus gourmands (sauf ceux à garder)
    ps aux | awk 'NR>1 && ($3 > 10 || $6 > 512000) {
        cmdline = ""
        for (i=11; i<=NF; i++) cmdline = cmdline " " $i
        
        # Vérifier si le processus doit être gardé
        keep = 0
        if (cmdline ~ /Cursor|ChatGPT|Perplexity|Visual Studio Code|Code|cursor|chatgpt|perplexity/) {
            keep = 1
        }
        
        if (!keep) {
            print $2  # PID
        }
    }' | while read pid; do
        if [[ -n "$pid" && "$pid" =~ ^[0-9]+$ ]]; then
            echo "🛑 Arrêt du processus PID $pid"
            kill -TERM "$pid" 2>/dev/null || true
            sleep 1
            # Si le processus ne répond pas, le tuer
            if kill -0 "$pid" 2>/dev/null; then
                echo "💀 Force kill du processus PID $pid"
                kill -KILL "$pid" 2>/dev/null || true
            fi
        fi
    done
    
    echo "✅ Processus gourmands arrêtés"
else
    echo "❌ Arrêt annulé"
fi

echo ""
echo "📊 Processus restants avec CPU > 5%:"
ps aux | awk 'NR>1 && $3 > 5 {
    cmdline = ""
    for (i=11; i<=NF; i++) cmdline = cmdline " " $i
    printf "PID: %s, CPU: %s%%, RAM: %.1fMB, CMD: %s\n", $2, $3, $6/1024, cmdline
}'
