Hi all,

Sharing an update on V3 TIM inspection and the 3D laser scanner selection.

Decision
We’ve selected Micro-Epsilon’s scanCONTROL LTT30x0-430 as the baseline scanner for V3 TIM inspection.

References

- Qual / trade study / findings: https://confluence.spacex.corp/spaces/~sfesseh/pages/6866203952/3D+Laser+Scanning+Qual+Findings
- PDR recap: https://jira.spacex.corp/browse/SLTOOL-27338

Why 3D scanning (vs. 2D vision)
3D scanning provides improved process control by enabling inspection degrees of freedom that 2D vision can’t support (true height/volume validation and more robust geometry checks). With this scanner we can support:

- XYZ measurement with ~±0.1 mm accuracy (application-dependent)
- Blob/line presence detection plus volume measurement, targeting ≤10% vs. expected volume

Key learnings / constraints

- Line-of-sight shadowing: Laser triangulation scanners can lose data when tall/steep features occlude the receiver line of sight. We evaluated partner-flagged geometries and did not find cases that would impact our current features, but this remains a known limitation to keep in mind for future designs.
- PCB scan speed: PCBs require longer exposure to produce a reliable scan. As a result, max scan speed for PCB surfaces is ~33 mm/s. Blob/line scans do not require long exposure and can run up to ~180 mm/s. The slower speed is therefore primarily a pre-dispense requirement. Even at the slower rate, we still gain a more accurate Z measurement than the V2 approach, which should reduce rework.

Next steps

- Finalize procurement / delivery plan for the LTT30x0-430
- Build out initial scan “recipes” for: (1) pre-dispense PCB scan, (2) post-dispense blob/line + volume verification
- Define acceptance criteria and update the V3 TIM inspection spec + MSA plan
- Track integration actions/risks in the PDR JIRA above

If you have concerns about the line-of-sight limitation for any upcoming geometry, or want specific features added to the scan recipe validation list, please reply here or comment in the JIRA.

Thanks,
[Your Name]