# Evolutionary-Computation on Pololu 3pi+ 32U4

**Thesis snapshot:** `v1.0-thesis`

Two Arduino firmwares for the Pololu **3pi+ 32U4** robot, plus simple host-side scripts.
**GA** and **DE** share the same firmware framework (I/O, runtime loop, logging); only the core optimizer differs.

This is the evolutionary computation island model framework, with the blue part representing replaceable algorithm modules driven by FSM
[![Evolutionary Computing Island Model Framework](docs/structurizr-1-c3-all.svg)](docs/structurizr-1-c3-all.svg)


## Contents

* **DE island/** — Differential Evolution firmware

  * `DE_recordingver.ino`, `ircomm_i2c.h`
* **GA island/** — Genetic Algorithm firmware

  * `GAr_recordingver.ino`, `ircomm_i2c.h`
* **MATLAB data recording script/** — `datarecord.m` (automates runs & logs serial CSV)
* **Plot script/** — `Plot.ipynb` (basic plotting)
* **Statistical testing/** — `Statistical_testing.ipynb` (basic statistical tests)
* **Data/** — place collected CSV here (raw)
* **docs/** — diagrams (`structurizr-1-c3-all.svg`, `structurizr-1-c3-all-key.svg`)

## Usage

### Firmware (Arduino)
- **Target board:** **Arduino Leonardo** (Pololu 3pi+ 32U4).
- Open the `.ino` in **GA island/** or **DE island/** with Arduino IDE 2.x → Board: *Arduino Leonardo* → select the port → Upload.
- Serial settings (both firmwares): **115200 baud**, LF line ending.

### Data collection (MATLAB)
- Run `MATLAB data recording script/datarecord.m` to drive the robot and record serial logs to CSV (place them under **Data/**).

### Analysis & plotting (Python)
- Two ways to run the Python notebooks:
  1. **Local/Jupyter** — open `Plot.ipynb` and `Statistical_testing.ipynb` (install deps as you like).
  2. **Google Colab** — upload the notebooks and your CSV logs and run them directly in Colab.





