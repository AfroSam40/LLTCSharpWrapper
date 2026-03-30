Progress
Stitching between scanners and scan passes is working.
Z-comp has been released on the KU gantry.
Key learnings so far
Stitching and Z-comp currently take about 30 seconds.
The primary constraint appears to be hardware performance.
Memory is the main bottleneck when retrieving profiles from the scanner controllers and constructing the point cloud.
After the data is loaded, both the CPU and GPU reach 100% utilization during stitching and Z-comp computation.
The original goal was to standardize on the same PC used between the dish and TIM scan systems. Based on current results, however, the TIM application will likely require a system with more memory and a higher-performance CPU/GPU.
Next steps
Evaluate and define the PC specifications required to achieve target inspection cycle times.
Continue monitoring system performance as vision inspection runs on Flight 1 parts.