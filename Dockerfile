FROM python:3.11-slim

# Installer dépendances système
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
    curl \
    && rm -rf /var/lib/apt/lists/*

# Créer répertoire de travail
WORKDIR /app

# Copier fichiers de dépendances
COPY pyproject.toml ./

# Installer dépendances Python
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -e ".[test]" && \
    pip install --no-cache-dir pytest-asyncio pytest-timeout

# Copier le code source
COPY src/ ./src/
COPY examples/ ./examples/

# Créer répertoires nécessaires
RUN mkdir -p artifacts logs

# Variables d'environnement
ENV MUJOCO_GL=disable
ENV DISPLAY=
ENV BBIA_DISABLE_AUDIO=1
ENV PYTHONPATH=/app/src

# Exposer les ports
EXPOSE 8000 8765

# Commande par défaut
CMD ["uvicorn", "src.bbia_sim.daemon.app.main:app", "--host", "0.0.0.0", "--port", "8000"]
