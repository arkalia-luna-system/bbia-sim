# 🤖 BBIA Reachy Mini Simulation

<div align="center">

[![Version](https://img.shields.io/badge/version-1.4.0-blue.svg)](https://github.com/arkalia-luna-system/bbia-sim)
[![Python](https://img.shields.io/badge/python-3.11%2B-blue.svg)](https://python.org)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![CI Status](https://github.com/arkalia-luna-system/bbia-sim/actions/workflows/ci.yml/badge.svg)](https://github.com/arkalia-luna-system/bbia-sim/actions/workflows/ci.yml)
[![Tests](https://img.shields.io/badge/tests-1743-brightgreen.svg)](https://github.com/arkalia-luna-system/bbia-sim/actions)
[![Code Quality](https://img.shields.io/badge/code%20quality-A%2B-brightgreen.svg)](https://github.com/arkalia-luna-system/bbia-sim)
[![SDK Conformity](https://img.shields.io/badge/SDK%20conformity-100%25-brightgreen.svg)](https://github.com/pollen-robotics/reachy_mini)
[![Documentation](https://img.shields.io/badge/docs-mkdocs-blue.svg)](docs/)
[![Coverage](https://img.shields.io/badge/coverage-tracked%20in%20Codecov-brightgreen)](https://app.codecov.io/gh/arkalia-luna-system/bbia-sim)

</div>

<div align="center">

## 🧠 Python Cognitive Engine for Reachy Mini Robot

### 🌟 Faithful Simulation • Advanced AI • 100% SDK Compliant

[![🚀 Quick Start](https://img.shields.io/badge/🚀-Quick%20Start-green)](#-quick-start)
[![📚 Documentation](https://img.shields.io/badge/📚-Documentation-blue)](docs/)
[![🤖 Try Demo](https://img.shields.io/badge/🤖-Try%20Demo-orange)](#-see-robot-in-3d)
[![🔧 Installation](https://img.shields.io/badge/🔧-Installation-purple)](#-quick-start)

</div>

---

## 📋 **IN 30 SECONDS**

<div align="center">

### 🎯 BBIA-SIM: Python Cognitive Engine for Reachy Mini

**12 robotic emotions** (6 official SDK + 6 extended) • **Faithful 3D simulation** • **Advanced AI**

![BBIA-SIM Reachy Mini Robot](./assets/images/robot_animation.gif)

</div>

### ✨ Key Features

<div align="center">

| 🎯 **Feature** | 📊 **Details** |
|:---:|:---|
| ✅ **Official SDK** | 100% compliant with Pollen Robotics |
| 🔄 **Unified Backend** | Simulation ↔ Real robot |
| 🧪 **Automated Tests** | 1,743 tests (unit + E2E) |
| 📚 **Documentation** | 209 Markdown files in `docs/` |
| ⚡ **Performance** | Optimized AI caches |
| 🔒 **Pro CI/CD** | Black, Ruff, MyPy, Bandit |
| 🌟 **Open Source** | MIT License |

</div>

## 📊 Statistics

- **Python source files**: 92 files (35,988 lines)
- **Test files**: 163 files (38,935 lines)
- **Test functions**: 1,804 tests identified
- **Collected tests**: 1,743 tests (GitHub Actions CI)
- **Coverage tracking**: maintained in [Codecov](https://app.codecov.io/gh/arkalia-luna-system/bbia-sim)
- **Core modules coverage**: ~50% (relevant measure)
- **BBIA modules**: 15+ specialized modules
- **Behaviors**: 21 intelligent behaviors
- **Documentation**: 209 Markdown files in `docs/`
- **Git commits**: 1,296+ commits

## 🏆 Code Quality

BBIA-SIM maintains strict quality standards:

- **Test coverage**: 80% minimum
- **Linting**: black, ruff, mypy, bandit
- **Python**: 3.11+
- **CI/CD**: GitHub Actions with automatic checks
- **Security**: Static analysis with Bandit
- **Performance**: Continuous optimizations and monitoring

**[🚀 Quick Start](#-quick-start)** | **[📚 Getting Started Guide](docs/guides/GUIDE_DEMARRAGE.md)** | **[🔬 Advanced Docs](docs/guides/GUIDE_AVANCE.md)**

---

Version 1.4.0 – Unified simulation/real backend, expanded tests and docs, active maintenance.

This project provides a faithful simulation of the Reachy Mini robot in MuJoCo, with integration of BBIA (Bio-Inspired Artificial Intelligence) modules and maintained alignment with the official Pollen Robotics ecosystem (tracked up to v1.7.0 on April 2026).

---

## 🚀 Quick Start

### Prerequisites

- Python 3.11+
- pip
- (Optional) Docker & Docker Compose for containerized environment

### Installation

#### Option 1: Standard Installation

```bash
# Clone repository
git clone https://github.com/arkalia-luna-system/bbia-sim.git
cd bbia-sim

# Install dependencies
pip install -e ".[test]"

# Run tests
pytest tests/ -v
```

#### Option 2: Docker Compose (Recommended)

```bash
# Clone repository
git clone https://github.com/arkalia-luna-system/bbia-sim.git
cd bbia-sim

# Start with Docker Compose
docker-compose up -d

# Check API health
curl http://localhost:8000/api/health
```

### Run Simulation

```bash
# Start API server
BBIA_TOKEN=your-secret-key uvicorn src.bbia_sim.daemon.app.main:app --port 8000

# Access dashboard
open http://localhost:8000
```

---

## 🎯 Features

### 12 Robotic Emotions

- **6 Official SDK emotions**: neutral, happy, sad, angry, curious, excited
- **6 Extended emotions**: surprised, fearful, confused, determined, nostalgic, proud

### 21 Intelligent Behaviors

- **Base behaviors** (7): Wake Up, Greeting, Emotional Response, Vision Tracking, Conversation, Antenna Animation, Hide
- **Advanced behaviors** (14): Follow Face, Follow Object, Dance, Emotion Show, Photo Booth, Storytelling, Teaching, Game, Meditation, Exercise, Alarm Clock, Weather Report, News Reader, Music Reaction

### Advanced AI Integration

- **Computer Vision**: YOLOv8n (object detection), MediaPipe (face/pose detection), DeepFace (emotion recognition)
- **Speech**: Whisper (STT), pyttsx3 (TTS), VAD (Voice Activity Detection)
- **NLP**: Transformers, Phi-2/TinyLlama (LLM), sentence-transformers
- **Function Calling**: Automatic detection of 6 robot actions

### Unified Backend Architecture

- **Same code** for simulation and real hardware
- **RobotAPI abstraction** for seamless switching
- **100% SDK compliant** with Pollen Robotics official SDK

---

## 📚 Documentation

- **[Getting Started Guide](docs/guides/GUIDE_DEMARRAGE.md)**: Complete installation and first steps
- **[Advanced Guide](docs/guides/GUIDE_AVANCE.md)**: Advanced features and customization
- **[API Documentation](docs/reference/)**: Complete API reference
- **[Video Demonstration Guide](docs/guides/GUIDE_VIDEOS_DEMONSTRATION.md)**: How to create demo videos
- **[Community Guide](docs/community/GUIDE_COMMUNAUTE.md)**: How to contribute and share

---

## 🧪 Testing

```bash
# Run all tests
pytest tests/ --cov=src/bbia_sim --cov-report=html

# Run specific test suite
pytest tests/test_reachy_mini_full_conformity_official.py -v

# View coverage report
open htmlcov/index.html
```

**Test Results**:
- ✅ **1,743 tests collected** (1,805 total, 62 deselected)
- ✅ **47 SDK conformity tests** (100% passing)
- ✅ **Coverage tracked continuously in Codecov**

---

## 🤝 Contributing

We welcome contributions! See our [Contribution Guide](docs/community/CONTRIBUTION_GUIDE.md) for details.

**Good First Issues**: We have 4 "Good First Issues" for new contributors.

---

## 📄 License

MIT License - see [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **Pollen Robotics** for the official Reachy Mini SDK
- **MuJoCo** for the physics simulation engine
- **Hugging Face** for AI models and transformers
- **OpenAI** for Whisper speech recognition

---

## 📞 Contact

- **GitHub**: [arkalia-luna-system/bbia-sim](https://github.com/arkalia-luna-system/bbia-sim)
- **Issues**: [GitHub Issues](https://github.com/arkalia-luna-system/bbia-sim/issues)
- **Discussions**: [GitHub Discussions](https://github.com/arkalia-luna-system/bbia-sim/discussions)

---

**Made with ❤️ by Arkalia Luna System**

*Last updated: April 27, 2026*

