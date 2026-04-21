# Case 01: Max Range

This case focuses on maximum-range analysis after engine cutoff. It searches for the best launch or flight-path setting, compares numerical methods, and constructs the attack envelope over multiple headings.

## Main Outputs

- optimal trajectory
- speed-time history
- attack boundary
- method comparison data

## Run

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\build_solver.ps1
powershell -ExecutionPolicy Bypass -File .\scripts\run_solver.ps1
```

To open the 3D viewer:

```bat
scripts\run_osgearth.bat
```
