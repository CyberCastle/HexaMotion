# TODOs Completados (OpenSHC_GAP_REPORT)

Fecha: 2026-02-10

## Implementaciones y cierres

- Pose con validacion y sentinel indefinido en [src/robot_model.h](src/robot_model.h).
- Estimacion de fuerza por Jacobiano en `Leg` y telemetria de esfuerzos en [src/leg.cpp](src/leg.cpp) y [src/locomotion_system.cpp](src/locomotion_system.cpp).
- Entrada cartesiana por pierna (poses manuales) en [src/state_controller.h](src/state_controller.h) y [src/locomotion_system.cpp](src/locomotion_system.cpp).
- Equivalencia de admittance delta documentada en [docs/OpenSHC_GAP_REPORT.md](docs/OpenSHC_GAP_REPORT.md).
- API publica de carga de soporte (`legsBearingLoad`) en [src/locomotion_system.h](src/locomotion_system.h).
- Getter de pose corporal actual en [src/body_pose_controller.h](src/body_pose_controller.h) y [src/locomotion_system.h](src/locomotion_system.h).
- Seguimiento de velocidad/esfuerzo por articulacion en [src/leg.h](src/leg.h).
- Stubs de planificador en [src/state_controller.cpp](src/state_controller.cpp).
- Ajuste ligero de parametros en [src/locomotion_system.cpp](src/locomotion_system.cpp).
- Documentacion de omisiones y racionales en [docs/OpenSHC_GAP_REPORT.md](docs/OpenSHC_GAP_REPORT.md).

## No aplicable (por diseno)

- `AMBLE_GAIT` se mantiene no soportado por las restricciones de morfologia (ver [AGENTS.md](AGENTS.md)).

## Validacion pendiente

- Pruebas de validacion listadas en [docs/OpenSHC_GAP_REPORT.md](docs/OpenSHC_GAP_REPORT.md) no se ejecutaron en este cambio.
