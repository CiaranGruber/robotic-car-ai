# Yuhao's logbook evidence extracts

- `mpc-speed-metrics.csv` / `mpc-speed-summary.json`: unchanged 30-run historical comparison, including settings, versions and source hashes.
- `endpoint-metrics.csv` / `endpoint-summary.json`: unchanged 70-run historical endpoint experiment.
- `uturn-summary.json`: original controller/configuration, scope, hashes, varied-layout cases/summary and separate cap probes. Other development/negative-test batches are omitted.
- `cross-summary.json` / `t-summary.json`: unchanged historical 26 Oct simulation reports. Their marker mapping predates the grouped IDs.
- `marker-range.csv`: 41 policy-observation rows from the saved recognition analysis. Elapsed time resets per run. Repeated detections can share acquisition times.
- `post-reload-trace.csv`: 705 rows of action `i`, observation `i`, post-step memory `i+1` from complete prefix records of the 15:02:43 capture. It contains wheel speed, sensor age, normalised actions, camera-pan command, phase and stop reason. No missing feedback is filled with invented zero values.
- `post-reload-events.json`: the original two-run transition analysis, including capture completeness.
- `sources.json`: origins, SHA-256 checksums and extraction rules. Paths beginning `build/` identify ignored source captures on the author's development machine, not files required to view the submitted logbook.

These files record existing experiments. Adding this logbook did not rerun the experiments or establish new physical performance. Figures 7–8 can be redrawn with `../make_figures.py`; the earlier simulation plots retain their original bytes.
