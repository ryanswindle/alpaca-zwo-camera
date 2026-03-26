# ASCOM Alpaca Server for ZWO camera (libASICamera2)

A FastAPI-based server, implementing the ASCOM **ICameraV4** interface. Communication is via published ASICamera2 library,
which has been tested up to V1.41 2026-01-12.

---

## Implemented ICameraV4 capabilities as of this driver version

| Capability           | Supported |
|----------------------|-----------|
| BayerOffsetX         | ✔         |
| BayerOffsetY         | ✔         |
| BinX                 | ✔         |
| BinY                 | ✔         |
| CameraState          | ✔         |
| CameraXSize          | ✔         |
| CameraYSize          | ✔         |
| CanAbortExposure     | ✔         |
| CanAsymmetricBin     | ✘         |
| CanFastReadout       | ✘         |
| CanGetCoolerPower    | ✔         |
| CanPulseGuide        | ✔         |
| CanSetCCDTemperature | ✔         |
| CanStopExposure      | ✔         |
| CCDTemperature       | ✔         |
| CoolerOn             | ✔         |
| CoolerPower          | ✔         |
| ElectronsPerADU      | ✔         |
| ExposureMax          | ✔         |
| ExposureMin          | ✔         |
| ExposureResolution   | ✔         |
| FastReadout          | ✘         |
| FullWellCapacity     | ✘         |
| Gain                 | ✔         |
| GainMax              | ✔         |
| GainMin              | ✔         |
| Gains                | ✘         |
| HasShutter           | ✔         |
| HeatSinkTemperature  | ✘         |
| ImageArray           | ✔         |
| ImageReady           | ✔         |
| IsPulseGuiding       | ✘         |
| LastExposureDuration | ✔         |
| MaxADU               | ✔         |
| MaxBinX              | ✔         |
| MaxBinY              | ✔         |
| NumX                 | ✔         |
| NumY                 | ✔         |
| Offset               | ✔         |
| OffsetMax            | ✔         |
| OffsetMin            | ✔         |
| Offsets              | ✘         |
| PercentCompleted     | ✘         |
| PixelSizeX           | ✔         |
| PixelSizeY           | ✔         |
| ReadoutMode          | ✔         |
| ReadoutModes         | ✔         |
| SensorName           | ✔         |
| SensorType           | ✔         |
| SetCCDTemperature    | ✔         |
| StartX               | ✔         |
| StartY               | ✔         |
| SubExposureDuration  | ✘         |
| AbortExposure        | ✔         |
| PulseGuide           | ✔         |
| StartExposure        | ✔         |
| StopExposure         | ✔         |

Tested on the ZWO ASI178MM.

---

## Architecture

| File               | Purpose                                     |
|--------------------|---------------------------------------------|
| `main.py`          | FastAPI app, lifespan, router wiring        |
| `config.py`        | Pydantic config models, YAML loader         |
| `config.yaml`      | User-editable configuration                 |
| `camera.py`        | FastAPI router – ICameraV4 endpoints        |
| `camera_device.py` | Low-level libqhyccd driver                  |
| `libqhyccd.py`     | Wrappers to ASICamera2 library              |
| `management.py`    | `/management` Alpaca management endpoints   |
| `setup.py`         | `/setup` HTML stub pages                    |
| `discovery.py`     | UDP Alpaca discovery responder (port 32227) |
| `responses.py`     | Pydantic response models                    |
| `exceptions.py`    | ASCOM Alpaca error classes                  |
| `shr.py`           | Shared FastAPI dependencies / helpers       |
| `log.py`           | Loguru config + stdlib intercept handler    |
| `test.py`          | Quick smoke-test script                     |
| `requirements.txt` | Python package dependencies                 |
| `Dockerfile`       | Container build                             |

---

## Configuration

Edit `config.yaml` to match your camera setup. Example settings:

- `library`: Path to `libASICamera2.so`
- `devices[].defaults`: Default temperature, readout mode, binning, gain, offset, USB traffic

Camera properties (sensor size, pixel size, gain/offset ranges, exposure limits) are
**queried from the SDK at connection time** — no hardcoding required.

Multiple ZWO cameras can be registered by adding further entries under
`devices:` with distinct `device_number` values.

## Quick start

```bash
pip install -r requirements.txt
python main.py
```

The server starts on `0.0.0.0:5000` by default (configurable in `config.yaml`).

---

## Smoke test

```bash
# Requires hardware connected, i.e. will operate camera
python test.py
```

---

## Docker

```bash
docker build -t alpaca-zwo-camera .
docker run -d --name alpaca-zwo-camera \
    -privileged -v /dev/bus/usb:/dev/bus/usb \
    --network host \
    --restart unless-stopped \
    alpaca-zwo-camera
docker logs -f alpaca-zwo-camera
```
