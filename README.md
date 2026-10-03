# Heterogeneous UAV Formation SITL

Reproducible source, frozen configuration, analysis scripts, tests, and compact summary tables for the fixed-slot heterogeneous UAV continuous-turning study.

## Scope

This repository contains the engineering and analysis materials for the frozen ArduPilot SITL work domain. It does not contain the large raw BIN/tlog archives. The complete raw evidence is archived separately with run manifests and SHA256 checksums.

## Included

- `uav_formation/`: formation reference generation, progress planning, measured-feedback NMPC path, and fixed-wing guidance.
- `scripts/`: experiment launch, audit, calibration, analysis, and plotting scripts.
- `tests/`: unit and contract tests.
- `config/experiments/`: experiment and platform configurations.
- `config/v3/`: frozen v3 capability, measurement, acceptance, and formal-experiment protocols.
- `results/`: compact supplementary tables for selected formal runs and excluded attempts.
- `docs/`: engineering verification notes.

For manuscript writing and evidence boundaries, start with
[`docs/论文撰写移交说明_20261003.md`](docs/论文撰写移交说明_20261003.md).

## Reproduction boundary

The formal results correspond to the recorded project commit
`6a6da740e33be7408ce121167410c983c94344d3` and ArduPilot commit
`a82481310ba19fe013e4797ea05cf702e76836bb` (`ArduPilot-4.6.0-beta1-7947-ga82481310b`).
The results are SITL engineering evidence for the frozen model, parameters, control architecture, and task domain; they are not claims about a real aircraft physical envelope.

The complete raw logs, including `raw_evidence.tar.gz`, BIN, tlog, parameter snapshots, and per-run source snapshots, are distributed separately because of their size. Use the accompanying data manifest and SHA256 files to match raw runs to this repository.

## Environment

The experiment wrapper expects ROS 2 Humble and uses the pinned paper dependencies in `requirements-paper.txt`. Formal runs recorded Python 3.10.12, NumPy 1.21.5, SciPy 1.8.0, Clarabel 0.11.1, OSQP 1.0.5, pymavlink 2.4.49, and PyMuPDF 1.26.4.

## Data and privacy

Absolute workstation paths, raw flight archives, credentials, and unrelated build products are intentionally excluded. The complete data package should be shared through a research data archive or private storage, not committed to the Git history.
