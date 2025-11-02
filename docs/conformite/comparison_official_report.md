# 🔍 Rapport de Comparaison Exhaustive - BBIA vs Repo Officiel

**Date**: 1762020963.15

## 📊 Résumé Exécutif

- **Total différences**: 165
- **🔴 CRITICAL**: 0
- **🟠 HIGH**: 1
- **🟡 MEDIUM**: 138
- **🟢 LOW**: 1
- **ℹ️ INFO**: 25

## 📋 Checklist par Catégorie

### API (25 différences)

1. **HIGH** - Endpoint présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /stl/{filename}`
   - Correction: Ajouter endpoint GET /stl/{filename} si nécessaire
   - Statut: pending

2. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /demo/start`
   - Statut: pending

3. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /capabilities`
   - Statut: pending

4. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /battery`
   - Statut: pending

5. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /behaviors/available`
   - Statut: pending

6. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /goto_sleep`
   - Statut: pending

7. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /behaviors/execute`
   - Statut: pending

8. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /stl/{filename:path}`
   - Statut: pending

9. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /head`
   - Statut: pending

10. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /emotions/apply`
   - Statut: pending

11. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /sensors`
   - Statut: pending

12. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /custom`
   - Statut: pending

13. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /simulation/stop`
   - Statut: pending

14. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /temperature`
   - Statut: pending

15. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /home`
   - Statut: pending

16. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /emotions/available`
   - Statut: pending

17. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /simulation/start`
   - Statut: pending

18. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /position`
   - Statut: pending

19. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /wake_up`
   - Statut: pending

20. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /joints`
   - Statut: pending

21. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /joints`
   - Statut: pending

22. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `GET /demo/modes`
   - Statut: pending

23. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /goto_pose`
   - Statut: pending

24. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /emergency_stop`
   - Statut: pending

25. **INFO** - Endpoint présent dans BBIA mais absent dans repo officiel (peut être extension BBIA)
   - Fichier: `src/bbia_sim/daemon/app/routers/`
   - Endpoint: `POST /gripper/{side}`
   - Statut: pending

### Doc (1 différences)

1. **LOW** - Section 'Usage' présente dans README officiel mais absente dans BBIA
   - Fichier: `README.md`
   - Correction: Vérifier si section 'Usage' doit être ajoutée
   - Statut: pending

### Structure (121 différences)

1. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/zenoh_server.py`
   - Correction: Vérifier si nécessaire d'ajouter zenoh_server.py
   - Statut: pending

2. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/camera_opencv.py`
   - Correction: Vérifier si nécessaire d'ajouter camera_opencv.py
   - Statut: pending

3. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._video_udp.py`
   - Correction: Vérifier si nécessaire d'ajouter ._video_udp.py
   - Statut: pending

4. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._rerun.py`
   - Correction: Vérifier si nécessaire d'ajouter ._rerun.py
   - Statut: pending

5. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/constants.py`
   - Correction: Vérifier si nécessaire d'ajouter constants.py
   - Statut: pending

6. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/analytical_kinematics.py`
   - Correction: Vérifier si nécessaire d'ajouter analytical_kinematics.py
   - Statut: pending

7. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/manager.py`
   - Correction: Vérifier si nécessaire d'ajouter manager.py
   - Statut: pending

8. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._camera_base.py`
   - Correction: Vérifier si nécessaire d'ajouter ._camera_base.py
   - Statut: pending

9. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/bg_job_register.py`
   - Correction: Vérifier si nécessaire d'ajouter bg_job_register.py
   - Statut: pending

10. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._webrtc_client_gstreamer.py`
   - Correction: Vérifier si nécessaire d'ajouter ._webrtc_client_gstreamer.py
   - Statut: pending

11. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/audio_utils.py`
   - Correction: Vérifier si nécessaire d'ajouter audio_utils.py
   - Statut: pending

12. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._parse_urdf_for_kinematics.py`
   - Correction: Vérifier si nécessaire d'ajouter ._parse_urdf_for_kinematics.py
   - Statut: pending

13. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._goto.py`
   - Correction: Vérifier si nécessaire d'ajouter ._goto.py
   - Statut: pending

14. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._zenoh_server.py`
   - Correction: Vérifier si nécessaire d'ajouter ._zenoh_server.py
   - Statut: pending

15. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/app.py`
   - Correction: Vérifier si nécessaire d'ajouter app.py
   - Statut: pending

16. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._interpolation.py`
   - Correction: Vérifier si nécessaire d'ajouter ._interpolation.py
   - Statut: pending

17. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._zenoh_client.py`
   - Correction: Vérifier si nécessaire d'ajouter ._zenoh_client.py
   - Statut: pending

18. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/nn_kinematics.py`
   - Correction: Vérifier si nécessaire d'ajouter nn_kinematics.py
   - Statut: pending

19. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/audio_sounddevice.py`
   - Correction: Vérifier si nécessaire d'ajouter audio_sounddevice.py
   - Statut: pending

20. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._local_common_venv.py`
   - Correction: Vérifier si nécessaire d'ajouter ._local_common_venv.py
   - Statut: pending

21. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._app.py`
   - Correction: Vérifier si nécessaire d'ajouter ._app.py
   - Statut: pending

22. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/audio_base.py`
   - Correction: Vérifier si nécessaire d'ajouter audio_base.py
   - Statut: pending

23. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._camera_utils.py`
   - Correction: Vérifier si nécessaire d'ajouter ._camera_utils.py
   - Statut: pending

24. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/protocol.py`
   - Correction: Vérifier si nécessaire d'ajouter protocol.py
   - Statut: pending

25. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._camera_opencv.py`
   - Correction: Vérifier si nécessaire d'ajouter ._camera_opencv.py
   - Statut: pending

26. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/backend.py`
   - Correction: Vérifier si nécessaire d'ajouter backend.py
   - Statut: pending

27. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._backend.py`
   - Correction: Vérifier si nécessaire d'ajouter ._backend.py
   - Statut: pending

28. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/media_manager.py`
   - Correction: Vérifier si nécessaire d'ajouter media_manager.py
   - Statut: pending

29. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/interpolation.py`
   - Correction: Vérifier si nécessaire d'ajouter interpolation.py
   - Statut: pending

30. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._recorded_move.py`
   - Correction: Vérifier si nécessaire d'ajouter ._recorded_move.py
   - Statut: pending

31. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._hf_space.py`
   - Correction: Vérifier si nécessaire d'ajouter ._hf_space.py
   - Statut: pending

32. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._reachy_mini.py`
   - Correction: Vérifier si nécessaire d'ajouter ._reachy_mini.py
   - Statut: pending

33. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/local_common_venv.py`
   - Correction: Vérifier si nécessaire d'ajouter local_common_venv.py
   - Statut: pending

34. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._nn_kinematics.py`
   - Correction: Vérifier si nécessaire d'ajouter ._nn_kinematics.py
   - Statut: pending

35. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/hf_space.py`
   - Correction: Vérifier si nécessaire d'ajouter hf_space.py
   - Statut: pending

36. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/camera_utils.py`
   - Correction: Vérifier si nécessaire d'ajouter camera_utils.py
   - Statut: pending

37. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._webrtc_daemon.py`
   - Correction: Vérifier si nécessaire d'ajouter ._webrtc_daemon.py
   - Statut: pending

38. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/camera_base.py`
   - Correction: Vérifier si nécessaire d'ajouter camera_base.py
   - Statut: pending

39. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._analytical_kinematics.py`
   - Correction: Vérifier si nécessaire d'ajouter ._analytical_kinematics.py
   - Statut: pending

40. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._dependencies.py`
   - Correction: Vérifier si nécessaire d'ajouter ._dependencies.py
   - Statut: pending

41. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._constants.py`
   - Correction: Vérifier si nécessaire d'ajouter ._constants.py
   - Statut: pending

42. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/camera_constants.py`
   - Correction: Vérifier si nécessaire d'ajouter camera_constants.py
   - Statut: pending

43. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/video_udp.py`
   - Correction: Vérifier si nécessaire d'ajouter video_udp.py
   - Statut: pending

44. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._audio_base.py`
   - Correction: Vérifier si nécessaire d'ajouter ._audio_base.py
   - Statut: pending

45. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/abstract.py`
   - Correction: Vérifier si nécessaire d'ajouter abstract.py
   - Statut: pending

46. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._bg_job_register.py`
   - Correction: Vérifier si nécessaire d'ajouter ._bg_job_register.py
   - Statut: pending

47. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/audio_gstreamer.py`
   - Correction: Vérifier si nécessaire d'ajouter audio_gstreamer.py
   - Statut: pending

48. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/parse_urdf_for_kinematics.py`
   - Correction: Vérifier si nécessaire d'ajouter parse_urdf_for_kinematics.py
   - Statut: pending

49. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._abstract.py`
   - Correction: Vérifier si nécessaire d'ajouter ._abstract.py
   - Statut: pending

50. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/utils.py`
   - Correction: Vérifier si nécessaire d'ajouter utils.py
   - Statut: pending

51. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/reachy_mini.py`
   - Correction: Vérifier si nécessaire d'ajouter reachy_mini.py
   - Statut: pending

52. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/camera_gstreamer.py`
   - Correction: Vérifier si nécessaire d'ajouter camera_gstreamer.py
   - Statut: pending

53. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._audio_utils.py`
   - Correction: Vérifier si nécessaire d'ajouter ._audio_utils.py
   - Statut: pending

54. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/placo_kinematics.py`
   - Correction: Vérifier si nécessaire d'ajouter placo_kinematics.py
   - Statut: pending

55. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/webrtc_daemon.py`
   - Correction: Vérifier si nécessaire d'ajouter webrtc_daemon.py
   - Statut: pending

56. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/zenoh_client.py`
   - Correction: Vérifier si nécessaire d'ajouter zenoh_client.py
   - Statut: pending

57. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/dependencies.py`
   - Correction: Vérifier si nécessaire d'ajouter dependencies.py
   - Statut: pending

58. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/webrtc_client_gstreamer.py`
   - Correction: Vérifier si nécessaire d'ajouter webrtc_client_gstreamer.py
   - Statut: pending

59. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._protocol.py`
   - Correction: Vérifier si nécessaire d'ajouter ._protocol.py
   - Statut: pending

60. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._audio_sounddevice.py`
   - Correction: Vérifier si nécessaire d'ajouter ._audio_sounddevice.py
   - Statut: pending

61. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._audio_gstreamer.py`
   - Correction: Vérifier si nécessaire d'ajouter ._audio_gstreamer.py
   - Statut: pending

62. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/recorded_move.py`
   - Correction: Vérifier si nécessaire d'ajouter recorded_move.py
   - Statut: pending

63. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/.___init__.py`
   - Correction: Vérifier si nécessaire d'ajouter .___init__.py
   - Statut: pending

64. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._utils.py`
   - Correction: Vérifier si nécessaire d'ajouter ._utils.py
   - Statut: pending

65. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._placo_kinematics.py`
   - Correction: Vérifier si nécessaire d'ajouter ._placo_kinematics.py
   - Statut: pending

66. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._camera_gstreamer.py`
   - Correction: Vérifier si nécessaire d'ajouter ._camera_gstreamer.py
   - Statut: pending

67. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/rerun.py`
   - Correction: Vérifier si nécessaire d'ajouter rerun.py
   - Statut: pending

68. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._manager.py`
   - Correction: Vérifier si nécessaire d'ajouter ._manager.py
   - Statut: pending

69. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._media_manager.py`
   - Correction: Vérifier si nécessaire d'ajouter ._media_manager.py
   - Statut: pending

70. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/goto.py`
   - Correction: Vérifier si nécessaire d'ajouter goto.py
   - Statut: pending

71. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `src/._camera_constants.py`
   - Correction: Vérifier si nécessaire d'ajouter ._camera_constants.py
   - Statut: pending

72. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_collision.py`
   - Correction: Vérifier si nécessaire d'ajouter ._test_collision.py
   - Statut: pending

73. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_wireless.py`
   - Correction: Vérifier si nécessaire d'ajouter ._test_wireless.py
   - Statut: pending

74. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_app.py`
   - Correction: Vérifier si nécessaire d'ajouter test_app.py
   - Statut: pending

75. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_audio.py`
   - Correction: Vérifier si nécessaire d'ajouter test_audio.py
   - Statut: pending

76. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_import.py`
   - Correction: Vérifier si nécessaire d'ajouter ._test_import.py
   - Statut: pending

77. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_collision.py`
   - Correction: Vérifier si nécessaire d'ajouter test_collision.py
   - Statut: pending

78. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_daemon.py`
   - Correction: Vérifier si nécessaire d'ajouter ._test_daemon.py
   - Statut: pending

79. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_analytical_kinematics.py`
   - Correction: Vérifier si nécessaire d'ajouter ._test_analytical_kinematics.py
   - Statut: pending

80. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_analytical_kinematics.py`
   - Correction: Vérifier si nécessaire d'ajouter test_analytical_kinematics.py
   - Statut: pending

81. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._main.py`
   - Correction: Vérifier si nécessaire d'ajouter ._main.py
   - Statut: pending

82. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_video.py`
   - Correction: Vérifier si nécessaire d'ajouter test_video.py
   - Statut: pending

83. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_app.py`
   - Correction: Vérifier si nécessaire d'ajouter ._test_app.py
   - Statut: pending

84. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_daemon.py`
   - Correction: Vérifier si nécessaire d'ajouter test_daemon.py
   - Statut: pending

85. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_placo.py`
   - Correction: Vérifier si nécessaire d'ajouter ._test_placo.py
   - Statut: pending

86. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_video.py`
   - Correction: Vérifier si nécessaire d'ajouter ._test_video.py
   - Statut: pending

87. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/main.py`
   - Correction: Vérifier si nécessaire d'ajouter main.py
   - Statut: pending

88. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_audio.py`
   - Correction: Vérifier si nécessaire d'ajouter ._test_audio.py
   - Statut: pending

89. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/.___init__.py`
   - Correction: Vérifier si nécessaire d'ajouter .___init__.py
   - Statut: pending

90. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_import.py`
   - Correction: Vérifier si nécessaire d'ajouter test_import.py
   - Statut: pending

91. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_wireless.py`
   - Correction: Vérifier si nécessaire d'ajouter test_wireless.py
   - Statut: pending

92. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_placo.py`
   - Correction: Vérifier si nécessaire d'ajouter test_placo.py
   - Statut: pending

93. **INFO** - Dossier scripts absent dans repo officiel (OK si normal)
   - Fichier: `scripts/`
   - Statut: pending

94. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/rerun_viewer.py`
   - Correction: Vérifier si nécessaire d'ajouter rerun_viewer.py
   - Statut: pending

95. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._compare_recordings.py`
   - Correction: Vérifier si nécessaire d'ajouter ._compare_recordings.py
   - Statut: pending

96. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/gstreamer_client.py`
   - Correction: Vérifier si nécessaire d'ajouter gstreamer_client.py
   - Statut: pending

97. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._sound_play.py`
   - Correction: Vérifier si nécessaire d'ajouter ._sound_play.py
   - Statut: pending

98. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._reachy_compliant_demo.py`
   - Correction: Vérifier si nécessaire d'ajouter ._reachy_compliant_demo.py
   - Statut: pending

99. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/sound_doa.py`
   - Correction: Vérifier si nécessaire d'ajouter sound_doa.py
   - Statut: pending

100. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._mini_head_position_gui.py`
   - Correction: Vérifier si nécessaire d'ajouter ._mini_head_position_gui.py
   - Statut: pending

101. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/compare_recordings.py`
   - Correction: Vérifier si nécessaire d'ajouter compare_recordings.py
   - Statut: pending

102. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/sound_play.py`
   - Correction: Vérifier si nécessaire d'ajouter sound_play.py
   - Statut: pending

103. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/body_yaw_test.py`
   - Correction: Vérifier si nécessaire d'ajouter body_yaw_test.py
   - Statut: pending

104. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._sound_record.py`
   - Correction: Vérifier si nécessaire d'ajouter ._sound_record.py
   - Statut: pending

105. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/mini_head_position_gui.py`
   - Correction: Vérifier si nécessaire d'ajouter mini_head_position_gui.py
   - Statut: pending

106. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._body_yaw_test.py`
   - Correction: Vérifier si nécessaire d'ajouter ._body_yaw_test.py
   - Statut: pending

107. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/joy_controller.py`
   - Correction: Vérifier si nécessaire d'ajouter joy_controller.py
   - Statut: pending

108. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._mini_body_yaw_gui.py`
   - Correction: Vérifier si nécessaire d'ajouter ._mini_body_yaw_gui.py
   - Statut: pending

109. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/compare_placo_nn_kin.py`
   - Correction: Vérifier si nécessaire d'ajouter compare_placo_nn_kin.py
   - Statut: pending

110. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._sound_doa.py`
   - Correction: Vérifier si nécessaire d'ajouter ._sound_doa.py
   - Statut: pending

111. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/gravity_compensation_direct_control.py`
   - Correction: Vérifier si nécessaire d'ajouter gravity_compensation_direct_control.py
   - Statut: pending

112. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._compare_placo_nn_kin.py`
   - Correction: Vérifier si nécessaire d'ajouter ._compare_placo_nn_kin.py
   - Statut: pending

113. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/reachy_compliant_demo.py`
   - Correction: Vérifier si nécessaire d'ajouter reachy_compliant_demo.py
   - Statut: pending

114. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._measure_tracking.py`
   - Correction: Vérifier si nécessaire d'ajouter ._measure_tracking.py
   - Statut: pending

115. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._joy_controller.py`
   - Correction: Vérifier si nécessaire d'ajouter ._joy_controller.py
   - Statut: pending

116. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/measure_tracking.py`
   - Correction: Vérifier si nécessaire d'ajouter measure_tracking.py
   - Statut: pending

117. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._gravity_compensation_direct_control.py`
   - Correction: Vérifier si nécessaire d'ajouter ._gravity_compensation_direct_control.py
   - Statut: pending

118. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/sound_record.py`
   - Correction: Vérifier si nécessaire d'ajouter sound_record.py
   - Statut: pending

119. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._rerun_viewer.py`
   - Correction: Vérifier si nécessaire d'ajouter ._rerun_viewer.py
   - Statut: pending

120. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/._gstreamer_client.py`
   - Correction: Vérifier si nécessaire d'ajouter ._gstreamer_client.py
   - Statut: pending

121. **MEDIUM** - Fichier présent dans repo officiel mais absent dans BBIA
   - Fichier: `examples/mini_body_yaw_gui.py`
   - Correction: Vérifier si nécessaire d'ajouter mini_body_yaw_gui.py
   - Statut: pending

### Test (18 différences)

1. **MEDIUM** - Test ._test_wireless présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_wireless.py`
   - Correction: Vérifier si test ._test_wireless doit être ajouté
   - Statut: pending

2. **MEDIUM** - Test test_app présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_app.py`
   - Correction: Vérifier si test test_app doit être ajouté
   - Statut: pending

3. **MEDIUM** - Test ._test_audio présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_audio.py`
   - Correction: Vérifier si test ._test_audio doit être ajouté
   - Statut: pending

4. **MEDIUM** - Test test_daemon présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_daemon.py`
   - Correction: Vérifier si test test_daemon doit être ajouté
   - Statut: pending

5. **MEDIUM** - Test test_wireless présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_wireless.py`
   - Correction: Vérifier si test test_wireless doit être ajouté
   - Statut: pending

6. **MEDIUM** - Test ._test_placo présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_placo.py`
   - Correction: Vérifier si test ._test_placo doit être ajouté
   - Statut: pending

7. **MEDIUM** - Test ._test_collision présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_collision.py`
   - Correction: Vérifier si test ._test_collision doit être ajouté
   - Statut: pending

8. **MEDIUM** - Test test_video présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_video.py`
   - Correction: Vérifier si test test_video doit être ajouté
   - Statut: pending

9. **MEDIUM** - Test ._test_import présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_import.py`
   - Correction: Vérifier si test ._test_import doit être ajouté
   - Statut: pending

10. **MEDIUM** - Test ._test_video présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_video.py`
   - Correction: Vérifier si test ._test_video doit être ajouté
   - Statut: pending

11. **MEDIUM** - Test ._test_daemon présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_daemon.py`
   - Correction: Vérifier si test ._test_daemon doit être ajouté
   - Statut: pending

12. **MEDIUM** - Test ._test_app présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_app.py`
   - Correction: Vérifier si test ._test_app doit être ajouté
   - Statut: pending

13. **MEDIUM** - Test test_audio présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_audio.py`
   - Correction: Vérifier si test test_audio doit être ajouté
   - Statut: pending

14. **MEDIUM** - Test test_placo présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_placo.py`
   - Correction: Vérifier si test test_placo doit être ajouté
   - Statut: pending

15. **MEDIUM** - Test test_analytical_kinematics présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_analytical_kinematics.py`
   - Correction: Vérifier si test test_analytical_kinematics doit être ajouté
   - Statut: pending

16. **MEDIUM** - Test ._test_analytical_kinematics présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/._test_analytical_kinematics.py`
   - Correction: Vérifier si test ._test_analytical_kinematics doit être ajouté
   - Statut: pending

17. **MEDIUM** - Test test_import présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_import.py`
   - Correction: Vérifier si test test_import doit être ajouté
   - Statut: pending

18. **MEDIUM** - Test test_collision présent dans repo officiel mais absent dans BBIA
   - Fichier: `tests/test_collision.py`
   - Correction: Vérifier si test test_collision doit être ajouté
   - Statut: pending

## ✅ Actions Recommandées

### Priorité CRITICAL

### Priorité HIGH
- [ ] Endpoint présent dans repo officiel mais absent dans BBIA
