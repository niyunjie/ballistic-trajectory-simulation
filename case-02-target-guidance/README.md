# Case 02: Target Guidance

This case extends the baseline solver to target-point matching. The optimization objective is changed from maximum range to minimum miss distance, and the guidance search includes heading adjustment and control tuning.

## Main Outputs

- target-matching trajectory
- miss-distance evaluation
- method comparison data
- 3D globe visualization with target and closest-approach markers

## Run

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\build_solver.ps1
powershell -ExecutionPolicy Bypass -File .\scripts\run_solver.ps1
```

To open the 3D viewer:

```bat
scripts\run_osgearth.bat
```
