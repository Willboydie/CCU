# LiDAR Scanning Host

Host-side tools for talking to the two-axis LiDAR gimbal over USB serial (460800 8N1). The library encodes host commands, parses device telemetry (`0x55` control at 1 kHz, `0xAA` status at 10 Hz), and drives a Dear ImGui transceiver UI.

The wire protocol is documented in [`docs/HOST_PROTOCOL.md`](docs/HOST_PROTOCOL.md).

## Prerequisites

- **CMake** 3.16 or newer
- A **C++20** compiler (Visual Studio 2022 on Windows, or GCC/Clang on Linux)
- **Boost** headers (Asio). Header-only is enough; this project does not link Boost libraries.
- **OpenGL** (system OpenGL on Windows; `libgl1-mesa-dev` on Debian/Ubuntu)
- **Git** (CMake FetchContent clones GLFW 3.4 and Dear ImGui v1.91.8 on first configure)

Point CMake at Boost with either:

```text
-DBOOST_ROOT=C:\Users\You\boost_1_92_0
```

or an environment variable named `BOOST_ROOT`.

## Build

From the repository root:

```powershell
cmake -S . -B build -DBOOST_ROOT="C:\Users\You\boost_1_92_0"
cmake --build build --config Release
```

On Linux / Ninja (single-config generators), drop `--config Release` and optionally pass `-DCMAKE_BUILD_TYPE=Release` at configure time.

That produces these executables:

| Target | Role |
|--------|------|
| `protocol_test` | Offline protocol unit tests (no hardware) |
| `scan_path_test` | Offline unit tests for the raster + infinity path |
| `lidar_transceiver` | Main GUI: connect, send commands, view telemetry |
| `scan_path_sim` | Offline GUI: simulate and plot raster + infinity scan paths |
| `lidar_serial_pipeline` | Console listener that auto-probes serial ports |

With Visual Studio (the default on this machine), they land in `build\Release\`. With Ninja or Makefiles they land in `build\`.

Rebuild a single target with:

```powershell
cmake --build build --config Release --target protocol_test
cmake --build build --config Release --target scan_path_test
cmake --build build --config Release --target lidar_transceiver
cmake --build build --config Release --target scan_path_sim
```

## Run tests

`protocol_test` checks packed packet sizes, command encoding (including the `TARGET_X_SET` example in the host protocol doc), and the device-stream parser. It does not open a serial port. `scan_path_test` checks ALPHA gap scaling for the raster + infinity path.

```powershell
.\build\Release\protocol_test.exe
.\build\Release\scan_path_test.exe
```

On Linux:

```bash
./build/protocol_test
./build/scan_path_test
```

Success prints `protocol tests passed` / `scan path tests passed` and exits 0. Failures print `FAIL: ...` and a non-zero exit status.

## Run the main program

`lidar_transceiver` is the interactive host. Pick a COM port in the UI, or pass one on the command line:

```powershell
.\build\Release\lidar_transceiver.exe
.\build\Release\lidar_transceiver.exe COM4
```

On Linux the device is typically `/dev/ttyACM0` or `/dev/ttyUSB0`:

```bash
./build/lidar_transceiver /dev/ttyACM0
```

The window lets you connect/disconnect, send protocol commands, and watch control/status counts. Close the window or press Ctrl+C to quit.

The device must already be enumerating as USB CDC. Baud is 460800; USB CDC mostly ignores it, but the host still opens at that rate.

## Scan path simulator

`scan_path_sim` is an offline GUI (no hardware). Enter INC, DELTA, ALPHA, GAP0, AMPX, AMPY, click **Set**, then **Go** to plot an infinity Lissajous (`freqX=2`, `freqY=4`) whose centre crawls a raster. The Lissajous is sampled every time step; **DELTA** (>1) is how many of those samples occur between centre updates. Vertical pitch starts at **GAP0**; **ALPHA** is added after each row. The path runs until the beam hits the top of the Y angular bounds. **Reset** clears the plot.

```powershell
.\build\Release\scan_path_sim.exe
```

## Console telemetry (optional)

`lidar_serial_pipeline` opens a port (or scans available ports) and prints decoded telemetry until Ctrl+C:

```powershell
.\build\Release\lidar_serial_pipeline.exe
.\build\Release\lidar_serial_pipeline.exe COM4
```

Control packets are sampled in the log (~10 Hz) so the console can keep up with the 1 kHz stream. Status and debug (`0xFF`) frames are printed as they arrive.
