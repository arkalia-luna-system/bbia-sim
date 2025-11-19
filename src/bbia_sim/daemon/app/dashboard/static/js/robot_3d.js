/**
 * Visualisation 3D du robot Reachy Mini avec Three.js
 * Animation selon état : awake, sleeping, error
 */

const robot3d = {
    // Three.js components
    scene: null,
    camera: null,
    renderer: null,
    robot: null,

    // Animation
    animationFrameId: null,
    currentState: 'stopped',

    // Configuration
    config: {
        cameraDistance: 5,
        cameraHeight: 2,
        rotationSpeed: 0.01
    },

    /**
     * Initialise la scène 3D
     */
    init: function () {
        console.log('🤖 Initialisation visualisation 3D robot...');

        const canvas = document.getElementById('robot-3d-canvas');
        if (!canvas) {
            console.warn('⚠️ Canvas robot-3d-canvas non trouvé');
            return;
        }

        // Vérifier que Three.js est chargé
        if (typeof THREE === 'undefined') {
            console.error('❌ Three.js non chargé. Ajouter le CDN dans base.html');
            return;
        }

        // Créer scène
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0xf9fafb); // Fond gris clair

        // Créer caméra
        const width = canvas.clientWidth || 400;
        const height = canvas.clientHeight || 400;
        this.camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
        this.camera.position.set(
            this.config.cameraDistance,
            this.config.cameraHeight,
            this.config.cameraDistance
        );
        this.camera.lookAt(0, 0, 0);

        // Créer renderer
        this.renderer = new THREE.WebGLRenderer({
            canvas: canvas,
            antialias: true
        });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(window.devicePixelRatio);

        // Ajouter lumière
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(5, 5, 5);
        this.scene.add(directionalLight);

        // Créer robot simplifié (géométrie basique en attendant modèle STL)
        this.createRobotPlaceholder();

        // Démarrer animation
        this.startAnimation();

        // Écouter changements d'état daemon
        this.setupStateListener();
    },

    /**
     * Crée un placeholder robot (géométrie simple)
     * TODO: Charger modèle STL réel
     */
    createRobotPlaceholder: function () {
        // Groupe pour le robot
        this.robot = new THREE.Group();

        // Corps (cylindre)
        const bodyGeometry = new THREE.CylinderGeometry(0.3, 0.3, 1.5, 16);
        const bodyMaterial = new THREE.MeshStandardMaterial({
            color: 0x008181 // Couleur conforme dashboard
        });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.position.y = 0.75;
        this.robot.add(body);

        // Tête (sphère)
        const headGeometry = new THREE.SphereGeometry(0.25, 16, 16);
        const headMaterial = new THREE.MeshStandardMaterial({
            color: 0x008181
        });
        const head = new THREE.Mesh(headGeometry, headMaterial);
        head.position.y = 1.75;
        this.robot.add(head);

        // Antennes (2 cylindres)
        for (let i = 0; i < 2; i++) {
            const antennaGeometry = new THREE.CylinderGeometry(0.02, 0.02, 0.3, 8);
            const antennaMaterial = new THREE.MeshStandardMaterial({
                color: 0x008181
            });
            const antenna = new THREE.Mesh(antennaGeometry, antennaMaterial);
            antenna.position.set(i * 0.15 - 0.075, 2.0, 0);
            this.robot.add(antenna);
        }

        this.scene.add(this.robot);
    },

    /**
     * Démarre l'animation
     */
    startAnimation: function () {
        if (this.animationFrameId) {
            cancelAnimationFrame(this.animationFrameId);
        }

        const animate = () => {
            this.update();
            this.animationFrameId = requestAnimationFrame(animate);
        };

        animate();
    },

    /**
     * Met à jour l'animation
     */
    update: function () {
        if (!this.robot) return;

        // Rotation lente selon état
        if (this.currentState === 'running' || this.currentState === 'starting') {
            this.robot.rotation.y += this.config.rotationSpeed;
        } else if (this.currentState === 'stopping') {
            this.robot.rotation.y += this.config.rotationSpeed * 0.5;
        }

        // Rendu
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        }
    },

    /**
     * Configure l'écoute des changements d'état
     */
    setupStateListener: function () {
        // Écouter changements via polling (à améliorer avec WebSocket)
        setInterval(() => {
            this.checkDaemonStatus();
        }, 1000);
    },

    /**
     * Vérifie le statut du daemon
     */
    checkDaemonStatus: async function () {
        try {
            const response = await fetch('/api/daemon/status');
            if (response.ok) {
                const data = await response.json();
                const newState = data.status || 'stopped';
                if (newState !== this.currentState) {
                    this.updateState(newState);
                }
            }
        } catch (error) {
            console.debug('Erreur récupération statut daemon:', error);
        }
    },

    /**
     * Met à jour l'état et l'animation
     */
    updateState: function (state) {
        this.currentState = state;

        if (!this.robot) return;

        // Changer couleur selon état
        const colorMap = {
            'running': 0x00ff00,    // Vert
            'starting': 0xffff00,   // Jaune
            'stopping': 0xff8800,   // Orange
            'stopped': 0x888888,    // Gris
            'error': 0xff0000       // Rouge
        };

        const color = colorMap[state] || 0x008181;

        // Appliquer couleur à tous les meshes
        this.robot.traverse((child) => {
            if (child instanceof THREE.Mesh) {
                child.material.color.setHex(color);
            }
        });
    },

    /**
     * Nettoie les ressources
     */
    dispose: function () {
        if (this.animationFrameId) {
            cancelAnimationFrame(this.animationFrameId);
        }

        if (this.renderer) {
            this.renderer.dispose();
        }
    }
};

// Initialisation au chargement
window.addEventListener('load', () => {
    // Attendre que Three.js soit chargé
    if (typeof THREE !== 'undefined') {
        robot3d.init();
    } else {
        console.warn('⚠️ Three.js non chargé. Attente...');
        // Réessayer après 1 seconde
        setTimeout(() => {
            if (typeof THREE !== 'undefined') {
                robot3d.init();
            }
        }, 1000);
    }
});

