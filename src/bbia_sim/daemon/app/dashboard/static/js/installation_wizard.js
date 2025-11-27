// Assistant d'Installation Interactif
const installationWizard = {
    currentStep: 1,
    totalSteps: 4,
    robotDetected: false,
    networkConfigured: false,
    sensorsTested: false,
    appsInstalled: false,

    init: function() {
        // Vérifier si c'est le premier lancement
        const firstLaunch = !localStorage.getItem('bbia-installation-complete');
        if (firstLaunch) {
            this.show();
        }

        // Événements
        document.getElementById('wizard-next')?.addEventListener('click', () => this.nextStep());
        document.getElementById('wizard-prev')?.addEventListener('click', () => this.prevStep());
        document.getElementById('skip-robot-detection')?.addEventListener('click', () => this.skipRobotDetection());
        document.getElementById('skip-network')?.addEventListener('click', () => this.skipNetwork());
        document.getElementById('run-sensor-tests')?.addEventListener('click', () => this.runSensorTests());
        document.getElementById('install-apps')?.addEventListener('click', () => this.installApps());
        document.getElementById('close-wizard')?.addEventListener('click', () => this.close());
        document.getElementById('test-network')?.addEventListener('click', () => this.testNetwork());

        // Démarrer la détection automatique du robot
        if (firstLaunch) {
            this.detectRobot();
        }
    },

    show: function() {
        const wizard = document.getElementById('installation-wizard');
        if (wizard) {
            wizard.classList.remove('hidden');
        }
    },

    hide: function() {
        const wizard = document.getElementById('installation-wizard');
        if (wizard) {
            wizard.classList.add('hidden');
        }
    },

    nextStep: function() {
        if (this.currentStep < this.totalSteps) {
            this.hideStep(this.currentStep);
            this.currentStep++;
            this.showStep(this.currentStep);
            this.updateNavigation();
        }
    },

    prevStep: function() {
        if (this.currentStep > 1) {
            this.hideStep(this.currentStep);
            this.currentStep--;
            this.showStep(this.currentStep);
            this.updateNavigation();
        }
    },

    showStep: function(step) {
        const stepElement = document.getElementById(`step-${step}`);
        if (stepElement) {
            stepElement.classList.remove('hidden');
        }
    },

    hideStep: function(step) {
        const stepElement = document.getElementById(`step-${step}`);
        if (stepElement) {
            stepElement.classList.add('hidden');
        }
    },

    updateNavigation: function() {
        const prevBtn = document.getElementById('wizard-prev');
        const nextBtn = document.getElementById('wizard-next');

        if (prevBtn) {
            prevBtn.classList.toggle('hidden', this.currentStep === 1);
        }

        if (nextBtn) {
            if (this.currentStep === this.totalSteps) {
                nextBtn.textContent = 'Terminer';
            } else {
                nextBtn.textContent = 'Suivant';
            }
        }
    },

    detectRobot: async function() {
        const statusEl = document.getElementById('robot-detection-status');
        try {
            // Appel API pour détecter le robot
            const response = await fetch('/api/daemon/status');
            const data = await response.json();

            if (data.robot_connected) {
                this.robotDetected = true;
                if (statusEl) {
                    statusEl.innerHTML = '<span class="text-green-600">✅ Robot détecté !</span>';
                }
            } else {
                if (statusEl) {
                    statusEl.innerHTML = '<span class="text-yellow-600">⚠️ Aucun robot détecté (Mode Simulation)</span>';
                }
            }
        } catch (error) {
            console.error('Erreur détection robot:', error);
            if (statusEl) {
                statusEl.innerHTML = '<span class="text-red-600">❌ Erreur de détection</span>';
            }
        }
    },

    skipRobotDetection: function() {
        this.robotDetected = false;
        this.nextStep();
    },

    testNetwork: async function() {
        const ssid = document.getElementById('wifi-ssid')?.value;
        const password = document.getElementById('wifi-password')?.value;

        if (!ssid) {
            alert('Veuillez entrer un SSID WiFi');
            return;
        }

        try {
            // Appel API pour tester le réseau
            const response = await fetch('/api/troubleshooting/test/network', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ host: '8.8.8.8' })
            });

            const data = await response.json();
            if (data.success) {
                this.networkConfigured = true;
                alert('✅ Connexion réseau testée avec succès');
            } else {
                alert('❌ Erreur de connexion réseau');
            }
        } catch (error) {
            console.error('Erreur test réseau:', error);
            alert('❌ Erreur lors du test réseau');
        }
    },

    skipNetwork: function() {
        this.networkConfigured = false;
        this.nextStep();
    },

    runSensorTests: async function() {
        const tests = ['camera', 'microphone', 'speaker'];
        
        for (const test of tests) {
            const statusEl = document.getElementById(`${test}-status`);
            if (statusEl) {
                statusEl.textContent = 'Test en cours...';
                statusEl.className = 'text-blue-500';
            }

            try {
                let endpoint = '';
                if (test === 'camera') {
                    endpoint = '/api/troubleshooting/test/camera';
                } else if (test === 'microphone') {
                    endpoint = '/api/troubleshooting/test/audio';
                } else {
                    // Speaker test via audio
                    endpoint = '/api/troubleshooting/test/audio';
                }

                const response = await fetch(endpoint, { method: 'POST' });
                const data = await response.json();

                if (statusEl) {
                    if (data.success) {
                        statusEl.textContent = '✅ OK';
                        statusEl.className = 'text-green-600';
                    } else {
                        statusEl.textContent = '❌ Erreur';
                        statusEl.className = 'text-red-600';
                    }
                }
            } catch (error) {
                console.error(`Erreur test ${test}:`, error);
                if (statusEl) {
                    statusEl.textContent = '❌ Erreur';
                    statusEl.className = 'text-red-600';
                }
            }
        }

        this.sensorsTested = true;
    },

    installApps: async function() {
        const appsToInstall = [];
        if (document.getElementById('install-vision')?.checked) {
            appsToInstall.push('bbia_vision');
        }
        if (document.getElementById('install-chat')?.checked) {
            appsToInstall.push('bbia_chat');
        }
        if (document.getElementById('install-emotions')?.checked) {
            appsToInstall.push('bbia_emotions');
        }

        for (const appName of appsToInstall) {
            try {
                const response = await fetch('/api/apps/install', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ name: appName, source_kind: 'local' })
                });

                const data = await response.json();
                console.log(`Installation ${appName}:`, data);
            } catch (error) {
                console.error(`Erreur installation ${appName}:`, error);
            }
        }

        this.appsInstalled = true;
        this.hideStep(4);
        this.showStep('complete');
        this.updateNavigation();
    },

    close: function() {
        localStorage.setItem('bbia-installation-complete', 'true');
        this.hide();
    }
};

// Initialiser au chargement
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => installationWizard.init());
} else {
    installationWizard.init();
}

