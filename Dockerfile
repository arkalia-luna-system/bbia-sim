# Dockerfile pour BBIA-SIM
FROM python:3.11-slim

# Variables d'environnement
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

# Installation des dépendances système
RUN apt-get update && apt-get install -y \
    libgl1-mesa-dri \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libglu1-mesa \
    portaudio19-dev \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Définir le répertoire de travail
WORKDIR /app

# Copier les fichiers de dépendances
COPY requirements.txt pyproject.toml ./

# Installer les dépendances Python
RUN pip install --no-cache-dir -r requirements.txt

# Copier le code source
COPY src/ ./src/
COPY scripts/ ./scripts/
COPY examples/ ./examples/

# Créer le répertoire logs
RUN mkdir -p logs

# Exposer le port de l'API
EXPOSE 8000

# Commande par défaut
CMD ["python", "-m", "uvicorn", "src.bbia_sim.daemon.app.main:app", "--host", "0.0.0.0", "--port", "8000"]
