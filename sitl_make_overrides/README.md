# SITL OOT Override

This folder contains host-side overrides that are bind-mounted into the `crazysim` container.

## What is overridden

- `CMakeLists.txt` -> mounted to:
  - `/CrazySim/crazyflie-firmware/sitl_make/CMakeLists.txt`

## Why

CrazySim launches the SITL executable `cf2` from `crazyflie-firmware/sitl_make/build`.
To run an Out-Of-Tree controller in SITL, the controller source must be compiled into this `cf2` binary.

## Integration details

The override `CMakeLists.txt`:

- Adds `/CrazySim/app_my_controller/src/controller_outoftree.c` to SITL sources.
- Enables `CONFIG_CONTROLLER_OOT` compile definitions.
- Includes deck interface paths needed by this CrazySim firmware tree.
- Emits this CMake message during configure:
  - `SITL OOT override enabled: using /CrazySim/app_my_controller/src/controller_outoftree.c`

## Compose mount

See `docker-compose.yaml` volume mapping:

- `./sitl_make_overrides/CMakeLists.txt:/CrazySim/crazyflie-firmware/sitl_make/CMakeLists.txt`
