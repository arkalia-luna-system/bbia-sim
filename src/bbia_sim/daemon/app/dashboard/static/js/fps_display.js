/**
 * Affichage FPS en temps réel
 * Couleur verte si >30 FPS, orange si <30 FPS
 */

const fpsDisplay = {
    fps: 60,
    lastTime: performance.now(),
    frameCount: 0,
    updateInterval: 1000, // Mise à jour toutes les secondes

    /**
     * Initialise l'affichage FPS
     */
    init: function () {
        const display = document.getElementById('fps-display');
        if (!display) {
            console.warn('⚠️ Élément fps-display non trouvé');
            return;
        }

        this.update();
    },

    /**
     * Met à jour l'affichage FPS
     */
    update: function () {
        const now = performance.now();
        const delta = now - this.lastTime;
        this.frameCount++;

        if (delta >= this.updateInterval) {
            this.fps = Math.round((this.frameCount * 1000) / delta);
            this.frameCount = 0;
            this.lastTime = now;

            const display = document.getElementById('fps-display');
            if (display) {
                display.textContent = `${this.fps} FPS`;

                // Changer couleur selon FPS
                if (this.fps >= 30) {
                    display.className = 'text-green-600';
                } else {
                    display.className = 'text-orange-600';
                }
            }
        }

        requestAnimationFrame(() => this.update());
    }
};

// Initialisation au chargement
window.addEventListener('load', () => {
    fpsDisplay.init();
});

