# Windows release builder

`build_release.ps1` assembles the offline Windows bundle used for GitHub Releases. It deliberately excludes generated archives and the local dependency cache from Git.

Before building, populate `packaging/cache/` with the signed Python installer and the exact wheels pinned in `requirements-runtime.txt`. The builder also expects the locally reviewed upstream model source directories referenced in the script. These inputs are not fetched automatically, so a release build cannot silently change dependencies or model revisions.

The builder checks the Python installer signature and hash, the verified firmware hash, text encoding, archive CRC, and known machine-specific identifiers. It creates a ZIP and adjacent `.zip.sha256` file under `dist/`.

The generated ZIP is a hardware-capable prerelease. Building it does not contact or operate a robot, serial adapter, gateway, or motor.
