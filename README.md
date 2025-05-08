# Motion

For the motion controller (Teensy) on the RamBOT.

## Prerequisites

PlatformIO on VSCode or standalone

Flatbuffers: https://github.com/google/flatbuffers

## Other Stuff

The comm protocol is defined in `MotionProtocol.fbs`, to generate the FlatBuffer output files run:

```bash
flatc --cpp -o include/ MotionProtocol.fbs
flatc --python --python-typing -o ../platform/ MotionProtocol.fbs
```

this assumes you have this repository located adjacent to the [platform](https://github.com/RAMBotsCSU/platform) repository to also generate the python side.

```
├── RamBOTs
│   ├── motion
│   ├── platform
```

---

#### Updating FlatBuffers

There is a helper script `update_flatbuffers.sh` which should theoretically update the FlatBuffer header files when a new version is released.
