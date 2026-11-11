# Scenes and matching motion clips

| ID | Backend scene | Leader clip | Paired U-Arm/CR10A clip | Purpose |
|---|---|---|---|---|
| 00 | mapping_check | clips/00_joint_mapping_sweep.json | paired_clips/00_joint_mapping_sweep.json | isolate J1..J6 and verify strict index mapping |
| 01 | tabletop | clips/01_tabletop_reach.json | paired_clips/01_tabletop_reach.json | tabletop reach preview |
| 02 | pickplace | clips/02_pickplace_motion.json | paired_clips/02_pickplace_motion.json | approach/close/lift/transfer/open timing |
| 03 | cloth | clips/03_cloth_fold_motion.json | paired_clips/03_cloth_fold_motion.json | fold-like motion; MuJoCo has deformable flex cloth |
| 04 | tabletop | clips/04_full_chain_demo.json | paired_clips/04_full_chain_demo.json | coordinated all-six-joint motion |

The paired clips are generated from the single source of truth `config/joint_mapping.json` by `tools/export_paired_clips.py`.
