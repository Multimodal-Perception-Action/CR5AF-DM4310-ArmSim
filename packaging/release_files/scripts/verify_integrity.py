"""Verify packaged payload hashes. Does not contact any hardware or service."""
import argparse
import hashlib
from pathlib import Path


def verify(root):
    root = root.resolve()
    manifest = root / "SHA256SUMS.txt"
    errors = []
    seen = set()
    for line in manifest.read_text(encoding="utf-8-sig").splitlines():
        expected, sep, relative = line.partition(" *")
        if not sep or len(expected) != 64 or any(c not in "0123456789abcdef" for c in expected):
            errors.append("Invalid manifest entry")
            continue
        target = (root / relative).resolve()
        if not target.is_relative_to(root) or relative in seen:
            errors.append(f"Invalid or duplicated path: {relative}")
            continue
        seen.add(relative)
        if not target.is_file():
            errors.append(f"Missing: {relative}")
            continue
        with target.open("rb") as stream:
            actual = hashlib.file_digest(stream, "sha256").hexdigest()
        if actual != expected:
            errors.append(f"Changed: {relative}")
    if errors:
        for error in errors:
            print(error)
        return 1
    print(f"PASS: {len(seen)} packaged file hashes match. Locally generated files are not checked.")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", nargs="?", type=Path, default=Path(__file__).resolve().parents[1])
    raise SystemExit(verify(parser.parse_args().root))
