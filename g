Hi all,

Wanted to broadcast an update regarding V3 TIM Inspection. The scanner we’ve decided on is Micro-Epsilon’s LLT30x0-430. Full details of qual, trade studies, and findings can be found here:
https://confluence.spacex.corp/spaces/sfesseha/pages/6866203952/3D+Laser+Scanning+Qual+Findings

PDR Recap: https://jira.spacex.corp/browse/SLTOOL-27338

3D laser scanning will give greater process control by allowing for a greater inspection degree of freedom that 2D vision is unable to do. Scanner will give us the following capabilities:

- XYZ comp with an accuracy of ±0.1 mm
- Blob/line presence detection with volume measurements to confirm within 10% of expected

Key learnings to note:

- Laser triangulation 3D scanners can lose data when tall or steep features block the receiver line of sight. Depending on the feature height, blob or line can be partially or fully shadowed. We have tested various potentially problematic features that partners have pointed out and have not found cases where this will affect us.
- PCBs will require a long exposure time for a reliable scan. As a result, max speed we’ll be able to scan is 33 mm/s. Blobs/Lines don’t require a long exposure time for a reliable scan and can be scanned at a much higher speed (180 mm/s) so the slower speed for PCBs is only required for predisepse inspection. Slow scanning will still give us a more accurate Z comp than V2 implementation which will improve rework stats.

Thanks,
Simon