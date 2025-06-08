# AGENT Instructions

Este archivo define las pautas para colaborar en HexaMotion.

## Estilo de código
- Utilizar C++17.
- Indentación de 4 espacios sin tabulaciones.
- Colocar la llave de apertura en la misma línea que la declaración.
- Documentar funciones públicas con comentarios estilo Doxygen (`/** ... */`).

## Pruebas
- Antes de enviar cambios, ejecutar las pruebas unitarias.
- Instalar la dependencia Eigen ejecutando `tests/setup.sh` si es necesario.
- Construir las pruebas con `make` dentro del directorio `tests`.

```bash
cd tests
./setup.sh
make
```

Cada ejecutable de prueba puede ejecutarse de forma individual.

## Mensajes de commit
- Usar tiempo imperativo en inglés. Ejemplos: "Add new gait option".
- Mantener el resumen por debajo de 72 caracteres.

## Solicitudes de extracción
- Incluir un resumen de los cambios realizados.
- Mencionar cualquier limitación conocida o pasos adicionales.

