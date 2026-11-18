# Documentation

The guides follow the order in which the stack is built and used. Read 1–2 first; 3–8
follow the pipeline from an empty track to a racing car; 9–10 cover running and scoring
experiments.

| # | Guide | Read it to |
|---|---|---|
| 1 | [Setup](01_setup.md) | install the simulator, this repository and its dependencies; build |
| 2 | [Architecture](02_architecture.md) | understand the packages, topics, frames and design decisions |
| 3 | [Mapping](03_mapping.md) | build a map of a track with Cartographer |
| 4 | [Track extraction](04_track_extraction.md) | turn a map into a centreline with wall widths |
| 5 | [Global planning](05_global_planning.md) | optimise a raceline offline |
| 6 | [Localization](06_localization.md) | choose and configure a localizer |
| 7 | [Local planning](07_local_planning.md) | keep the car clear of what the map got wrong |
| 8 | [Control](08_control.md) | track the path; reactive controllers |
| 9 | [Running simulations](09_running_simulations.md) | run any combination with one command |
| 10 | [Evaluation tools](10_evaluation.md) | run folder contents, lap logger, plots |
| 11 | [Troubleshooting](11_troubleshooting.md) | fix known problems |

Related:

- [environment_changes.md](../environment_changes.md): every change made to the external simulator.
- Each package in [`code/`](../code/README.md) has its own README with its full interface and
  parameters; the guides link to them instead of repeating them.
