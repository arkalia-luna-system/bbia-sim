const updateMoveItems = async (dataset) => {
    fetch(`/api/move/recorded-move-datasets/list/${dataset}`)
        .then(response => response.json())
        .then(data => {
            // Update the UI with the list of recorded moves
            const moveSelectToggle = document.getElementById('move-select-toggle');
            moveSelectToggle.innerHTML = '';

            data.forEach(moveName => {
                const option = document.createElement('option');
                option.value = moveName;
                option.textContent = moveName;
                moveSelectToggle.appendChild(option);
            });

        })
        .catch(error => {
            console.error(`Error fetching recorded moves for dataset '${dataset}':`, error);
        });
};

const movePlayer = {
    playing: false,
    currentMove: null,

    playRecordedMove: async (dataset, move) => {
        console.log(`Requesting play move '${move}' from dataset '${dataset}'`);

        // movePlayer.playing = true;
        // movePlayer.updateUI();

        await fetch(`/api/move/play/recorded-move-dataset/${dataset}/${move}`, {
            method: 'POST'
        }).then(response => {
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json();
        }
        ).then(data => {
            movePlayer.currentMove = data.uuid;
        }).catch(error => {
            console.error(`Error playing move '${move}' from dataset '${dataset}':`, error);
            // movePlayer.playing = false;
            // movePlayer.currentMove = null;
            // movePlayer.updateUI();
        });
    },

    stopMove: async () => {
        console.log(`Requesting stop of current move`);

        await fetch(`/api/move/stop`, {
            method: 'POST',
            body: JSON.stringify({ uuid: movePlayer.currentMove }),
            headers: {
                'Content-Type': 'application/json'
            }
        });
    },

    updateUI: () => {
        const movePlayBtn = document.getElementById('move-play-btn');
        const moveStopBtn = document.getElementById('move-stop-btn');

        if (movePlayer.playing) {
            movePlayBtn.disabled = true;
            moveStopBtn.disabled = false;
        } else {
            movePlayBtn.disabled = false;
            moveStopBtn.disabled = true;
        }
    },

    checkMoveStatus: async () => {
        let ws = new WebSocket(`ws://${window.location.host}/api/move/ws/updates`);

        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'move_started') {
                movePlayer.playing = true;
                movePlayer.currentMove = data.move_id;
            }
            else if (data.type === 'move_completed') {
                movePlayer.playing = false;
                movePlayer.currentMove = null;
            }
            else if (data.type === 'move_failed' || data.type === 'move_cancelled') {
                movePlayer.playing = false;
                movePlayer.currentMove = null;
            }

            movePlayer.updateUI();
        };

        ws.onclose = () => {
            console.log('Move status WebSocket closed, reconnecting in 1 second...');
            setTimeout(() => {
                movePlayer.checkMoveStatus();
            }, 1000);
        };
    }
};

// AMÉLIORATION: Charger dynamiquement les datasets depuis /discover
// Mentionnée dans docs/audit/DECISION_FINAL_AMELIORATIONS.md (🟡 2)
const loadAvailableDatasets = async () => {
    try {
        const response = await fetch('/api/move/recorded-move-datasets/discover');
        if (!response.ok) {
            console.error(`Error fetching datasets: ${response.status}`);
            return; // Fallback vers datasets hardcodés dans HTML
        }
        
        const datasets = await response.json();
        const moveDatasetToggle = document.getElementById('move-dataset-toggle');
        
        // Vider le select
        moveDatasetToggle.innerHTML = '';
        
        // Ajouter chaque dataset trouvé
        datasets.forEach(dataset => {
            const option = document.createElement('option');
            option.value = dataset;
            // Afficher nom lisible (extraire après le dernier "/")
            const displayName = dataset.split('/').pop().replace(/-/g, ' ').replace(/_/g, ' ');
            option.textContent = displayName.charAt(0).toUpperCase() + displayName.slice(1);
            moveDatasetToggle.appendChild(option);
        });
        
        console.log(`✅ Loaded ${datasets.length} datasets dynamically`);
        
        // Initialiser les moves pour le premier dataset
        if (datasets.length > 0) {
            updateMoveItems(datasets[0]);
        }
    } catch (error) {
        console.error('Error loading datasets:', error);
        // Fallback: laisser les datasets hardcodés dans HTML
    }
};

window.addEventListener('DOMContentLoaded', (event) => {
    // Charger dynamiquement les datasets disponibles
    loadAvailableDatasets();
    
    const moveDatasetToggle = document.getElementById('move-dataset-toggle');

    moveDatasetToggle.addEventListener('change', (_) => {
        const selectedDataset = moveDatasetToggle.value;
        updateMoveItems(selectedDataset);
    });

    const movePlayBtn = document.getElementById('move-play-btn');
    movePlayBtn.addEventListener('click', async () => {
        const selectedDataset = moveDatasetToggle.value;
        const moveSelectToggle = document.getElementById('move-select-toggle');
        const selectedMove = moveSelectToggle.value;

        await movePlayer.playRecordedMove(selectedDataset, selectedMove);
    });

    const moveStopBtn = document.getElementById('move-stop-btn');
    moveStopBtn.addEventListener('click', async () => {
        await movePlayer.stopMove();
    });

    movePlayer.checkMoveStatus();
    movePlayer.updateUI();
});