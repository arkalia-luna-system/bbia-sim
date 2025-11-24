/**
 * Visualisation waveform audio en temps réel
 * Utilise Web Audio API pour analyser et afficher les waveforms
 */

const waveform = {
    // Canvas et contexte
    speakerCanvas: null,
    microphoneCanvas: null,
    speakerCtx: null,
    microphoneCtx: null,

    // Audio context et analysers
    audioContext: null,
    speakerAnalyser: null,
    microphoneAnalyser: null,

    // Animation frame ID
    animationFrameId: null,

    // Configuration
    config: {
        updateRate: 30, // FPS (limité à 30 pour performance)
        barWidth: 2,
        barGap: 1,
        barColor: '#008181', // Couleur conforme dashboard officiel
        backgroundColor: '#f3f4f6'
    },

    /**
     * Initialise les waveforms
     */
    init: function () {
        console.log('🌊 Initialisation waveforms...');

        // Récupérer les canvas
        this.speakerCanvas = document.getElementById('speaker-waveform');
        this.microphoneCanvas = document.getElementById('microphone-waveform');

        if (!this.speakerCanvas || !this.microphoneCanvas) {
            console.warn('⚠️ Canvas waveforms non trouvés');
            return;
        }

        // Récupérer les contextes
        this.speakerCtx = this.speakerCanvas.getContext('2d');
        this.microphoneCtx = this.microphoneCanvas.getContext('2d');

        // Initialiser Web Audio API si disponible
        if (window.AudioContext || window.webkitAudioContext) {
            try {
                this.audioContext = new (window.AudioContext || window.webkitAudioContext)();
                this.initAnalysers();
            } catch (error) {
                console.warn('⚠️ Web Audio API non disponible:', error);
                // Fallback: afficher waveform statique
                this.drawStaticWaveform();
            }
        } else {
            console.warn('⚠️ Web Audio API non supporté');
            this.drawStaticWaveform();
        }

        // Démarrer l'animation
        this.startAnimation();
    },

    /**
     * Initialise les analysers audio
     */
    initAnalysers: function () {
        if (!this.audioContext) return;

        // Créer analysers pour speaker et microphone
        // Note: En production, ces analysers seraient connectés aux vraies sources audio
        // Pour l'instant, on simule avec des données aléatoires

        this.speakerAnalyser = this.audioContext.createAnalyser();
        this.speakerAnalyser.fftSize = 256;
        this.speakerAnalyser.smoothingTimeConstant = 0.8;

        this.microphoneAnalyser = this.audioContext.createAnalyser();
        this.microphoneAnalyser.fftSize = 256;
        this.microphoneAnalyser.smoothingTimeConstant = 0.8;

        // Note: Fonctionnalité future - Connexion sources audio réelles
        // Les sources audio pourront être connectées via:
        // - WebSocket pour streaming audio depuis le serveur
        // - MediaStream API pour accès direct au microphone
        // Pour l'instant, on simule avec des données pour la démonstration
    },

    /**
     * Dessine une waveform statique (fallback)
     */
    drawStaticWaveform: function () {
        if (this.speakerCtx && this.speakerCanvas) {
            this.drawWaveform(this.speakerCtx, this.speakerCanvas, []);
        }
        if (this.microphoneCtx && this.microphoneCanvas) {
            this.drawWaveform(this.microphoneCtx, this.microphoneCanvas, []);
        }
    },

    /**
     * Démarre l'animation des waveforms
     */
    startAnimation: function () {
        if (this.animationFrameId) {
            cancelAnimationFrame(this.animationFrameId);
        }

        const draw = () => {
            this.update();
            this.animationFrameId = requestAnimationFrame(draw);
        };

        draw();
    },

    /**
     * Arrête l'animation
     */
    stopAnimation: function () {
        if (this.animationFrameId) {
            cancelAnimationFrame(this.animationFrameId);
            this.animationFrameId = null;
        }
    },

    /**
     * Met à jour les waveforms (appelé à chaque frame)
     */
    update: function () {
        // Limiter la fréquence de mise à jour (30 FPS)
        const now = Date.now();
        if (!this.lastUpdate) {
            this.lastUpdate = now;
        }
        const elapsed = now - this.lastUpdate;
        const frameInterval = 1000 / this.config.updateRate;

        if (elapsed < frameInterval) {
            return;
        }
        this.lastUpdate = now;

        // Récupérer les données audio
        let speakerData = [];
        let microphoneData = [];

        if (this.speakerAnalyser) {
            const bufferLength = this.speakerAnalyser.frequencyBinCount;
            const dataArray = new Uint8Array(bufferLength);
            this.speakerAnalyser.getByteFrequencyData(dataArray);
            speakerData = Array.from(dataArray);
        } else {
            // Simulation: données aléatoires pour démo
            speakerData = this.generateSimulatedData();
        }

        if (this.microphoneAnalyser) {
            const bufferLength = this.microphoneAnalyser.frequencyBinCount;
            const dataArray = new Uint8Array(bufferLength);
            this.microphoneAnalyser.getByteFrequencyData(dataArray);
            microphoneData = Array.from(dataArray);
        } else {
            // Simulation: données aléatoires pour démo
            microphoneData = this.generateSimulatedData();
        }

        // Dessiner les waveforms
        if (this.speakerCtx && this.speakerCanvas) {
            this.drawWaveform(this.speakerCtx, this.speakerCanvas, speakerData);
        }
        if (this.microphoneCtx && this.microphoneCanvas) {
            this.drawWaveform(this.microphoneCtx, this.microphoneCanvas, microphoneData);
        }
    },

    /**
     * Génère des données simulées pour la démo
     */
    generateSimulatedData: function () {
        const data = [];
        const numBars = 64;
        for (let i = 0; i < numBars; i++) {
            // Générer des valeurs aléatoires avec variation douce
            const value = Math.random() * 128 + Math.sin(i * 0.1) * 30;
            data.push(Math.max(0, Math.min(255, value)));
        }
        return data;
    },

    /**
     * Dessine une waveform sur un canvas
     */
    drawWaveform: function (ctx, canvas, data) {
        const width = canvas.width;
        const height = canvas.height;

        // Effacer le canvas
        ctx.fillStyle = this.config.backgroundColor;
        ctx.fillRect(0, 0, width, height);

        if (!data || data.length === 0) {
            return;
        }

        // Calculer la largeur des barres
        const barWidth = this.config.barWidth;
        const barGap = this.config.barGap;
        const totalBarWidth = barWidth + barGap;
        const numBars = Math.floor(width / totalBarWidth);
        const dataStep = Math.max(1, Math.floor(data.length / numBars));

        // Dessiner les barres
        ctx.fillStyle = this.config.barColor;

        for (let i = 0; i < numBars; i++) {
            const dataIndex = Math.min(i * dataStep, data.length - 1);
            const value = data[dataIndex] || 0;

            // Normaliser la valeur (0-255 -> 0-height)
            const barHeight = (value / 255) * height * 0.8; // 80% de la hauteur max

            // Centrer verticalement
            const x = i * totalBarWidth;
            const y = (height - barHeight) / 2;

            ctx.fillRect(x, y, barWidth, barHeight);
        }
    }
};

// Exporter pour utilisation globale
window.waveform = waveform;

