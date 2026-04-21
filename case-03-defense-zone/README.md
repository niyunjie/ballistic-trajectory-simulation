# Case 03: Defense Zone

This case adds a simplified defense-interception layer on top of the target-guidance workflow. A trajectory may be kinematically reachable but still be considered ineffective if a defense site can intercept it within the allowed delay, range, speed, and altitude limits.

## Main Outputs

- reachable and blocked target solutions
- effective attack boundary
- intercepted attack boundary
- defense-aware visualization on the 3D globe

## Run

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\build_solver.ps1
powershell -ExecutionPolicy Bypass -File .\scripts\run_solver.ps1
```

To open the 3D viewer:

```bat
scripts\run_osgearth.bat
```
