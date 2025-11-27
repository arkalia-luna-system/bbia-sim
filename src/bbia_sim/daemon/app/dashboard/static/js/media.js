/**
 * Gestion des contrôles media (Speaker, Microphone, Camera)
 * Conforme au style du dashboard officiel Reachy Mini
 */

const media = {
    // État actuel
    currentStatus: {
        speaker: { volume: 0.5, active: true },
        microphone: { volume: 0.5, active: true },
        camera: { enabled: false, active: false }
    },

    // Debounce pour éviter trop de requêtes
    updateTimeouts: {},

    /**
     * Initialise les contrôles media
     */
    init: async function() {
        console.log('🎵 Initialisation contrôles media...');

        // Récupérer statut initial
        await this.getStatus();

        // Configurer les event listeners
        this.setupEventListeners();

        // Initialiser les waveforms
        if (window.waveform) {
            window.waveform.init();
        }
    },

    /**
     * Configure les event listeners pour les contrôles
     */
    setupEventListeners: function() {
        // Slider volume speaker
        const speakerSlider = document.getElementById('speaker-volume');
        if (speakerSlider) {
            speakerSlider.addEventListener('input', (e) => {
                const value = parseInt(e.target.value, 10);
                this.updateSpeakerVolume(value);
            });
        }

        // Slider volume microphone
        const microphoneSlider = document.getElementById('microphone-volume');
        if (microphoneSlider) {
            microphoneSlider.addEventListener('input', (e) => {
                const value = parseInt(e.target.value, 10);
                this.updateMicrophoneVolume(value);
            });
        }

        // Toggle camera
        const cameraToggle = document.getElementById('camera-toggle');
        if (cameraToggle) {
            cameraToggle.addEventListener('change', (e) => {
                this.toggleCamera(e.target.checked);
            });
        }
    },

    /**
     * Récupère le statut actuel des media
     */
    getStatus: async function() {
        try {
            const response = await fetch('/development/api/media/status');
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            const data = await response.json();
            
            if (data) {
                this.currentStatus = {
                    speaker: {
                        volume: (data.speaker?.volume || 0.5) * 100,
                        active: data.speaker?.active !== false
                    },
                    microphone: {
                        volume: (data.microphone?.volume || 0.5) * 100,
                        active: data.microphone?.active !== false
                    },
                    camera: {
                        enabled: data.camera?.enabled === true,
                        active: data.camera?.active === true
                    }
                };

                this.updateUI();
            }
        } catch (error) {
            console.error('Erreur récupération statut media:', error);
            // Fallback: utiliser valeurs par défaut
            this.updateUI();
        }
    },

    /**
     * Met à jour l'interface utilisateur avec les valeurs actuelles
     */
    updateUI: function() {
        // Speaker
        const speakerSlider = document.getElementById('speaker-volume');
        const speakerValue = document.getElementById('speaker-volume-value');
        const speakerStatus = document.getElementById('speaker-status');
        
        if (speakerSlider) {
            speakerSlider.value = this.currentStatus.speaker.volume;
        }
        if (speakerValue) {
            speakerValue.textContent = Math.round(this.currentStatus.speaker.volume);
        }
        if (speakerStatus) {
            speakerStatus.textContent = this.currentStatus.speaker.active ? 'Active' : 'Inactive';
            speakerStatus.className = this.currentStatus.speaker.active 
                ? 'text-sm text-green-600' 
                : 'text-sm text-gray-500';
        }

        // Microphone
        const microphoneSlider = document.getElementById('microphone-volume');
        const microphoneValue = document.getElementById('microphone-volume-value');
        const microphoneStatus = document.getElementById('microphone-status');
        
        if (microphoneSlider) {
            microphoneSlider.value = this.currentStatus.microphone.volume;
        }
        if (microphoneValue) {
            microphoneValue.textContent = Math.round(this.currentStatus.microphone.volume);
        }
        if (microphoneStatus) {
            microphoneStatus.textContent = this.currentStatus.microphone.active ? 'Active' : 'Inactive';
            microphoneStatus.className = this.currentStatus.microphone.active 
                ? 'text-sm text-green-600' 
                : 'text-sm text-gray-500';
        }

        // Camera
        const cameraToggle = document.getElementById('camera-toggle');
        const cameraToggleLabel = document.getElementById('camera-toggle-label');
        const cameraStatus = document.getElementById('camera-status');
        
        if (cameraToggle) {
            cameraToggle.checked = this.currentStatus.camera.enabled;
        }
        if (cameraToggleLabel) {
            cameraToggleLabel.textContent = this.currentStatus.camera.enabled ? 'ON' : 'OFF';
        }
        if (cameraStatus) {
            cameraStatus.textContent = this.currentStatus.camera.active ? 'Active' : 'Inactive';
            cameraStatus.className = this.currentStatus.camera.active 
                ? 'text-sm text-green-600' 
                : 'text-sm text-gray-500';
        }
    },

    /**
     * Met à jour le volume du speaker
     */
    updateSpeakerVolume: function(value) {
        // Clamp entre 0 et 100
        const clampedValue = Math.max(0, Math.min(100, value));
        
        // Mise à jour immédiate de l'UI
        const speakerValue = document.getElementById('speaker-volume-value');
        if (speakerValue) {
            speakerValue.textContent = Math.round(clampedValue);
        }

        // Debounce: envoyer la requête après 200ms d'inactivité
        if (this.updateTimeouts.speaker) {
            clearTimeout(this.updateTimeouts.speaker);
        }

        this.updateTimeouts.speaker = setTimeout(async () => {
            try {
                const volumeNormalized = clampedValue / 100.0; // Convertir en 0.0-1.0
                
                const response = await fetch('/development/api/media/speaker/volume', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ volume: volumeNormalized })
                });

                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }

                const data = await response.json();
                if (data.success) {
                    this.currentStatus.speaker.volume = clampedValue;
                    console.log(`✅ Volume speaker mis à jour: ${clampedValue}%`);
                }
            } catch (error) {
                console.error('Erreur mise à jour volume speaker:', error);
            }
        }, 200);
    },

    /**
     * Met à jour le volume du microphone
     */
    updateMicrophoneVolume: function(value) {
        // Clamp entre 0 et 100
        const clampedValue = Math.max(0, Math.min(100, value));
        
        // Mise à jour immédiate de l'UI
        const microphoneValue = document.getElementById('microphone-volume-value');
        if (microphoneValue) {
            microphoneValue.textContent = Math.round(clampedValue);
        }

        // Debounce: envoyer la requête après 200ms d'inactivité
        if (this.updateTimeouts.microphone) {
            clearTimeout(this.updateTimeouts.microphone);
        }

        this.updateTimeouts.microphone = setTimeout(async () => {
            try {
                const volumeNormalized = clampedValue / 100.0; // Convertir en 0.0-1.0
                
                const response = await fetch('/development/api/media/microphone/volume', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ volume: volumeNormalized })
                });

                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }

                const data = await response.json();
                if (data.success) {
                    this.currentStatus.microphone.volume = clampedValue;
                    console.log(`✅ Volume microphone mis à jour: ${clampedValue}%`);
                }
            } catch (error) {
                console.error('Erreur mise à jour volume microphone:', error);
            }
        }, 200);
    },

    /**
     * Toggle la caméra ON/OFF
     */
    toggleCamera: async function(enabled) {
        try {
            const response = await fetch('/development/api/media/camera/toggle', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ enabled: enabled })
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data = await response.json();
            if (data.success) {
                this.currentStatus.camera.enabled = enabled;
                this.currentStatus.camera.active = enabled && (data.camera?.active === true);
                this.updateUI();
                console.log(`✅ Caméra ${enabled ? 'activée' : 'désactivée'}`);
            }
        } catch (error) {
            console.error('Erreur toggle caméra:', error);
            // Revert toggle en cas d'erreur
            const cameraToggle = document.getElementById('camera-toggle');
            if (cameraToggle) {
                cameraToggle.checked = !enabled;
            }
        }
    }
};

// Initialisation au chargement de la page
window.addEventListener('load', () => {
    media.init();
});

