// Gestion de l'installation PWA
// Version: 1.0.0

let deferredPrompt = null;
let installButton = null;

// Écouter l'événement beforeinstallprompt
window.addEventListener('beforeinstallprompt', (e) => {
    // Empêcher l'affichage automatique du prompt
    e.preventDefault();
    // Stocker l'événement pour l'utiliser plus tard
    deferredPrompt = e;
    // Afficher le bouton d'installation
    showInstallButton();
});

// Fonction pour afficher le bouton d'installation
function showInstallButton() {
    if (!installButton) {
        // Créer le bouton s'il n'existe pas
        installButton = document.createElement('button');
        installButton.id = 'pwa-install-button';
        installButton.className = 'fixed bottom-4 right-4 bg-indigo-600 hover:bg-indigo-700 text-white font-semibold py-3 px-6 rounded-lg shadow-lg z-50 flex items-center gap-2 transition-all duration-200';
        installButton.innerHTML = `
            <span class="text-2xl">📱</span>
            <span>Installer l'app</span>
        `;
        installButton.addEventListener('click', installPWA);

        // Ajouter au body
        document.body.appendChild(installButton);
    }

    // Afficher le bouton
    installButton.style.display = 'flex';
}

// Fonction pour masquer le bouton
function hideInstallButton() {
    if (installButton) {
        installButton.style.display = 'none';
    }
}

// Fonction pour installer la PWA
async function installPWA() {
    if (!deferredPrompt) {
        // Si pas de prompt disponible, afficher instructions
        showInstallInstructions();
        return;
    }

    // Afficher le prompt d'installation
    deferredPrompt.prompt();

    // Attendre la réponse de l'utilisateur
    const { outcome } = await deferredPrompt.userChoice;

    if (outcome === 'accepted') {
        console.log('✅ PWA installée par l\'utilisateur');
        showToast('✅ Application installée avec succès !', 'success');
    } else {
        console.log('❌ Installation PWA refusée par l\'utilisateur');
    }

    // Réinitialiser
    deferredPrompt = null;
    hideInstallButton();
}

// Fonction pour afficher les instructions d'installation
function showInstallInstructions() {
    const isIOS = /iPad|iPhone|iPod/.test(navigator.userAgent);
    const isAndroid = /Android/.test(navigator.userAgent);

    let instructions = '';

    if (isIOS) {
        instructions = `
            <div class="pwa-instructions bg-blue-50 border border-blue-200 rounded-lg p-4 mb-4">
                <h3 class="font-semibold text-blue-900 mb-2">📱 Installation sur iOS</h3>
                <ol class="list-decimal list-inside space-y-1 text-sm text-blue-800">
                    <li>Appuyez sur le bouton <strong>Partager</strong> <span class="text-lg">📤</span></li>
                    <li>Sélectionnez <strong>"Sur l'écran d'accueil"</strong></li>
                    <li>Appuyez sur <strong>"Ajouter"</strong></li>
                </ol>
            </div>
        `;
    } else if (isAndroid) {
        instructions = `
            <div class="pwa-instructions bg-green-50 border border-green-200 rounded-lg p-4 mb-4">
                <h3 class="font-semibold text-green-900 mb-2">📱 Installation sur Android</h3>
                <ol class="list-decimal list-inside space-y-1 text-sm text-green-800">
                    <li>Appuyez sur le menu <span class="text-lg">⋮</span> (trois points)</li>
                    <li>Sélectionnez <strong>"Ajouter à l'écran d'accueil"</strong></li>
                    <li>Confirmez l'installation</li>
                </ol>
            </div>
        `;
    } else {
        instructions = `
            <div class="pwa-instructions bg-gray-50 border border-gray-200 rounded-lg p-4 mb-4">
                <h3 class="font-semibold text-gray-900 mb-2">📱 Installation sur Desktop</h3>
                <p class="text-sm text-gray-800">
                    Utilisez le menu de votre navigateur pour installer l'application.
                    <br>Chrome/Edge : Icône <span class="text-lg">➕</span> dans la barre d'adresse
                </p>
            </div>
        `;
    }

    // Afficher dans une notification ou modal
    showToast(instructions, 'info', 10000);
}

// Fonction pour afficher un toast
function showToast(message, type = 'info', duration = 3000) {
    // Créer l'élément toast
    const toast = document.createElement('div');
    toast.className = `fixed top-4 left-1/2 transform -translate-x-1/2 bg-white border border-gray-200 rounded-lg shadow-lg p-4 z-50 max-w-md`;

    const colors = {
        success: 'border-green-500 bg-green-50',
        error: 'border-red-500 bg-red-50',
        info: 'border-blue-500 bg-blue-50'
    };

    toast.className += ` ${colors[type] || colors.info}`;

    if (typeof message === 'string' && message.includes('<')) {
        toast.innerHTML = message;
    } else {
        toast.textContent = message;
    }

    // Ajouter au body
    document.body.appendChild(toast);

    // Supprimer après la durée
    setTimeout(() => {
        toast.style.opacity = '0';
        toast.style.transition = 'opacity 0.3s';
        setTimeout(() => {
            toast.remove();
        }, 300);
    }, duration);
}

// Vérifier si l'app est déjà installée
window.addEventListener('appinstalled', () => {
    console.log('✅ PWA installée');
    deferredPrompt = null;
    hideInstallButton();
    showToast('✅ Application installée avec succès !', 'success');
});

// Vérifier au chargement si l'app est déjà installée
if (window.matchMedia('(display-mode: standalone)').matches) {
    // L'app est déjà installée
    console.log('✅ PWA déjà installée');
    hideInstallButton();
}

// Exporter pour utilisation globale
window.installPWA = installPWA;
window.showInstallButton = showInstallButton;
window.hideInstallButton = hideInstallButton;

