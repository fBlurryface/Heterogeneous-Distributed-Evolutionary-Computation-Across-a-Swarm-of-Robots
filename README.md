# Evolutionary-Computation on Pololu 3pi+ 32U4

This repo contains two Arduino firmwares for the Pololu **3pi+ 32U4** robot and a small host-side toolkit. **GA** and **DE** share the same firmware framework (I/O, runtime loop, logging); only the core optimizer differs.

## Contents

* **arduino/ga/** — Genetic Algorithm firmware (Arduino C/C++)
* **arduino/de/** — Differential Evolution firmware (Arduino C/C++)
* **matlab/run\_trials.m** — Automates runs and collects serial logs to CSV
* **analysis/stats.ipynb** — Basic statistical analysis / figures
* **docs/code-map.svg** — Shared firmware framework diagram (shown below)

This is the evolutionary computation island model framework, with the blue part representing replaceable algorithm modules driven by FSM
[![Evolutionary Computing Island Model Framework](docs/structurizr-1-c3-all.svg)](docs/structurizr-1-c3-all.svg)

## Minimal layout

```
/
├─ arduino/ga/
├─ arduino/de/
├─ matlab/run_trials.m
├─ analysis/stats.ipynb
└─ docs/code-map.svg
```

## Usage

*Add your setup / flashing / run instructions here (ports, commands, parameters, data paths, etc.).*

## Reproducibility

Reference the frozen thesis tag:
`https://github.com/YOUR-OWNER/YOUR-REPO/tree/v1.0-thesis`
