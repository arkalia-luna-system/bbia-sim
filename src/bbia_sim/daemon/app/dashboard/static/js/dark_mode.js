/**
 * Gestion du Dark Mode pour le dashboard BBIA
 * - Toggle dark/light mode
 * - Persistance dans localStorage
 * - Support prefers-color-scheme
 */

(function () {
    'use strict';

    const STORAGE_KEY = 'bbia-dark-mode';
    const DARK_CLASS = 'dark';

    /**
     * Initialise le dark mode au chargement
     */
    function initDarkMode() {
        // Vérifier localStorage
        const saved = localStorage.getItem(STORAGE_KEY);
        let isDark = false;

        if (saved !== null) {
            // Utiliser préférence sauvegardée
            isDark = saved === 'true';
        } else {
            // Utiliser préférence système
            isDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
        }

        applyDarkMode(isDark);
        updateToggleButton(isDark);
    }

    /**
     * Applique le dark mode
     */
    function applyDarkMode(isDark) {
        if (isDark) {
            document.documentElement.classList.add(DARK_CLASS);
        } else {
            document.documentElement.classList.remove(DARK_CLASS);
        }
        localStorage.setItem(STORAGE_KEY, isDark.toString());
    }

    /**
     * Toggle dark mode
     */
    function toggleDarkMode() {
        const isDark = document.documentElement.classList.contains(DARK_CLASS);
        const newIsDark = !isDark;
        applyDarkMode(newIsDark);
        updateToggleButton(newIsDark);

        // Émettre événement pour autres composants
        window.dispatchEvent(new CustomEvent('darkmodechange', {
            detail: { isDark: newIsDark }
        }));
    }

    /**
     * Met à jour le bouton toggle
     */
    function updateToggleButton(isDark) {
        const toggleBtn = document.getElementById('dark-mode-toggle');
        if (!toggleBtn) return;

        if (isDark) {
            toggleBtn.innerHTML = '☀️ Light';
            toggleBtn.setAttribute('title', 'Passer en mode clair');
        } else {
            toggleBtn.innerHTML = '🌙 Dark';
            toggleBtn.setAttribute('title', 'Passer en mode sombre');
        }
    }

    /**
     * Crée le bouton toggle si absent
     */
    function createToggleButton() {
        // Vérifier si le bouton existe déjà
        if (document.getElementById('dark-mode-toggle')) {
            return;
        }

        // Créer le bouton
        const toggleBtn = document.createElement('button');
        toggleBtn.id = 'dark-mode-toggle';
        toggleBtn.className = 'fixed bottom-4 right-4 z-50 px-4 py-2 bg-gray-800 dark:bg-gray-200 text-white dark:text-gray-800 rounded-full shadow-lg hover:shadow-xl transition-all duration-200 text-sm font-medium';
        toggleBtn.setAttribute('aria-label', 'Toggle dark mode');

        // Ajouter au body
        document.body.appendChild(toggleBtn);

        // Attacher événement
        toggleBtn.addEventListener('click', toggleDarkMode);
    }

    // Initialiser quand le DOM est prêt
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', function () {
            initDarkMode();
            createToggleButton();
        });
    } else {
        initDarkMode();
        createToggleButton();
    }

    // Écouter changements système
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', function (e) {
        // Seulement si pas de préférence sauvegardée
        if (localStorage.getItem(STORAGE_KEY) === null) {
            applyDarkMode(e.matches);
            updateToggleButton(e.matches);
        }
    });
})();

