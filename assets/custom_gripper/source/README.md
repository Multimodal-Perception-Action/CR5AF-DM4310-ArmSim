# R66 / R67 incremental geometry — preliminary reference

This directory contains the R66 and R67 STEP/BREP files from the owner-provided incremental-parts handoff dated 2026-11-04.

## Scope and limitations

- These files are incremental geometry only, not a complete as-built gripper assembly.
- The source handoff excludes legacy/base parts, fasteners, and the full assembly context.
- The source note says this is not a frozen production release; material, tolerance, strength, and full-load validation remain open.
- Use these files for preliminary geometry review only. Do not derive final URDF joint placement, limits, or safety limits from them alone.
- No redistribution license was found in the supplied package. Confirm publication rights before pushing these files to the public repository.

The currently installed motor is DM-J4310-2EC V1.2. The available official local STEP model is labeled V1.1, so it has deliberately not been copied here as a V1.2 model. Verify the V1.2 mechanical dimensions before using any motor geometry.

A complete as-built assembly with the base, transmission, mounting interface, all moving parts, and component poses is still required for a faithful simulation model.