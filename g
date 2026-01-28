Below is a cleaned-up, “ready to paste” version that (1) tightens wording, (2) makes units/terms consistent, (3) finishes the unfinished ambient-light section, and (4) turns the notes into clear engineering conclusions.


---

Accuracy and repeatability

Requirements

Volume measurement error: ≤ 5%

Z compensation accuracy: within ±0.2 mm


Key drivers

Profile resolution (points per profile)

Sampling distance / scan interval (mm per profile)


Qualification test — setup

A 0.2 cc reference blob was measured using:

Profile resolution: 2048 points/profile

Sampling distance: 0.1 mm (baseline; varied in test)

Exposure time: 0.25 ms


True volume was calculated from mass and density.

Qualification test — results

Sampling distance (mm)	Measured volume (cc)	True volume (cc)	Error (cc)	Error (%)

0.1	0.203	0.216	0.013	6.019
0.2	0.192	0.216	0.024	11.111
0.3	0.178	0.216	0.038	17.593
0.4	0.124	0.216	0.092	42.593


Conclusion

0.1 mm sampling distance provides the best accuracy and is closest to the 5% target (≈6.0%).

Increasing sampling distance rapidly degrades volume accuracy (undersampling drives systematic under-measurement).


Recommendation

Use 0.1 mm as the default sampling distance for volume-critical measurements.

If the 5% requirement is strict (not “goal”), the next levers to close the remaining ~1% gap are typically: tighter calibration, improved segmentation/thresholding of the blob boundary, and/or slightly finer sampling (if cycle time allows).



---

Scan speed and profile frequency

Requirement

Scan time contribution should be <10% of total time.


Definitions and relationships

Scan time per profile = exposure time + processing time

Max profile frequency (Hz) is approximately limited by:


f_{max} \approx \frac{1}{t_{exposure} + t_{processing}}

v \;(\text{mm/s}) = f \;(\text{profiles/s}) \times \Delta x \;(\text{mm/profile})

1800 Hz → 180 mm/s

320 Hz → 32 mm/s


Primary drivers

1. Exposure time

Directly limits max frequency.

Strongly dependent on surface reflectivity and ambient light.



2. Processing time

Driven by field of view / region of interest (ROI) and algorithmic load.

Smaller ROI → less data per frame → faster conversion to a 2D profile.



3. Z field of view (Z FOV) / ROI

Reducing Z FOV (or ROI) can increase max frequency by reducing processing time—most noticeable at short exposure times.





---

Effects of ambient light on scan data (finished)

Ambient light can introduce artifacts when reflections are bright enough for the receiver to detect, effectively competing with (or washing out) the projected laser line. This is more prominent on reflective surfaces because specular reflections can:

Raise the background level, reducing laser-line contrast and making the line harder to segment reliably.

Create bright hotspots or saturation, which can pull the detected line position and produce Z spikes, discontinuities, or false edges.

Increase frame-to-frame variability, reducing repeatability.


Practical mitigations

Reduce ambient light (shrouds, curtains, enclosure, avoid direct overhead lighting/sunlight).

Use smaller exposure times when possible (higher contrast laser line → less time needed).

Constrain ROI / Z FOV to only what is needed (reduces processing time and sensitivity to stray reflections).

If available: optical filtering (narrowband), polarization, or geometry changes to avoid direct specular returns.



---

Qualification test — scan frequency vs exposure time

Qualification test — setup

Exposure time is an uncontrolled variable driven by material reflectivity. Testing was performed across an expected reflectivity range from PCB (least reflective) to polished aluminum (most reflective). The test was run on a dish scan to minimize ambient light.

Results — maximum achievable frequency

Exposure time (ms)	Max frequency (Hz), Z FOV @ 25% (92.5 mm)	Max frequency (Hz), Z FOV @ 13.5% (50 mm)

0.1	2000	3000
0.5	1500	1800
1	950	950
2	480	480
3	320	320
4	247	247
5	198	198
6	165	165


Interpretation / conclusion

At very short exposures (≤0.5 ms), smaller Z FOV increases max frequency (processing/ROI overhead matters).

Beyond ~1 ms, exposure time dominates and Z FOV has little impact (both columns converge).


Operational conclusion (as written, cleaned up)

Reflective surfaces can be scanned at up to 1800 Hz (≈ 180 mm/s at 0.1 mm sampling) assuming minimum Z FOV ≈ 13.5%.

Diffusive surfaces can be scanned at up to 320 Hz (≈ 32 mm/s at 0.1 mm sampling).



---

If you want, I can also convert this into a one-page “Executive Summary” + a shorter “Key Settings” box (recommended defaults + when to change exposure/ROI) so it reads like a finished test report rather than lab notes.