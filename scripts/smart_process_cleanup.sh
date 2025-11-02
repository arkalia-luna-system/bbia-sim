#!/bin/bash

# 🛑 Script intelligent pour arrêter les processus problématiques
# Garde les processus système macOS et les applications importantes

echo "🛑 Nettoyage intelligent des processus..."
echo "======================================="

# Fonction pour vérifier si un processus est critique
is_critical_process() {
    local cmdline="$1"
    local critical_processes=(
        "WindowServer" "kernel" "launchd" "mds_stores" "mdsync" 
        "fskitd" "UVFSService" "XprotectService"
        "Cursor" "ChatGPT" "Perplexity" "Visual Studio Code" "Code"
        "cursor" "chatgpt" "perplexity"
    )
    
    for process in "${critical_processes[@]}"; do
        if [[ "$cmdline" == *"$process"* ]]; then
            return 0  # Processus critique
        fi
    done
    return 1  # Processus non critique
}

echo "🔍 Recherche des processus problématiques (non critiques)..."
echo ""

# Identifier les processus gourmands non critiques
problematic_processes=()

ps aux | awk 'NR>1 && ($3 > 15 || $6 > 1000000) {
    cmdline = ""
    for (i=11; i<=NF; i++) cmdline = cmdline " " $i
    
    # Vérifier si le processus est critique
    critical = 0
    if (cmdline ~ /WindowServer|kernel|launchd|mds_stores|mdsync|fskitd|UVFSService|XprotectService|Cursor|ChatGPT|Perplexity|Visual Studio Code|Code|cursor|chatgpt|perplexity/) {
        critical = 1
    }
    
    if (!critical) {
        printf "%s|%s|%.1f|%s\n", $2, $3, $6/1024, cmdline
    }
}' | while IFS='|' read -r pid cpu ram cmdline; do
    if [[ -n "$pid" ]]; then
        echo "⚠️  PID $pid: ${cpu}% CPU, ${ram}MB RAM"
        echo "   CMD: ${cmdline:0:80}..."
        echo ""
    fi
done

echo "❓ Voulez-vous arrêter ces processus problématiques? (y/N)"
read -r response

if [[ "$response" =~ ^[Yy]$ ]]; then
    echo "🛑 Arrêt des processus problématiques..."
    
    # Arrêter les processus gourmands non critiques
    ps aux | awk 'NR>1 && ($3 > 15 || $6 > 1000000) {
        cmdline = ""
        for (i=11; i<=NF; i++) cmdline = cmdline " " $i
        
        # Vérifier si le processus est critique
        critical = 0
        if (cmdline ~ /WindowServer|kernel|launchd|mds_stores|mdsync|fskitd|UVFSService|XprotectService|Cursor|ChatGPT|Perplexity|Visual Studio Code|Code|cursor|chatgpt|perplexity/) {
            critical = 1
        }
        
        if (!critical) {
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
    
    echo "✅ Processus problématiques arrêtés"
else
    echo "❌ Arrêt annulé"
fi

echo ""
echo "📊 Processus restants avec CPU > 10%:"
ps aux | awk 'NR>1 && $3 > 10 {
    cmdline = ""
    for (i=11; i<=NF; i++) cmdline = cmdline " " $i
    printf "PID: %s, CPU: %s%%, RAM: %.1fMB, CMD: %s\n", $2, $3, $6/1024, cmdline
}'

echo ""
echo "💡 Pour arrêter les processus BBIA spécifiquement:"
echo "   python scripts/process_manager.py stop"
echo ""
echo "💡 Pour voir le statut des processus BBIA:"
echo "   python scripts/process_manager.py status"
