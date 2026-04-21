Run from the case root in MATLAB:

```matlab
addpath(fullfile(pwd, 'scripts', 'matlab'));
plot_exported_results(fullfile(pwd, 'apps', 'osgearth_viewer', 'data', 'exports'));
```

To process a single export folder:

```matlab
plot_exported_results(fullfile(pwd, 'apps', 'osgearth_viewer', 'data', 'exports', 'your-export-folder'));
```
