"""Verify that a release is safe for Windows ZIP extractors and UTF-8 tools."""

from __future__ import annotations

import argparse
from pathlib import Path
import zipfile


TEXT_SUFFIXES = {
    ".bat", ".cmd", ".cpp", ".csv", ".h", ".ini", ".json", ".md",
    ".ps1", ".py", ".txt", ".urdf", ".xml", ".yaml", ".yml",
}
TEXT_NAMES = {"LICENSE", "NOTICE"}
FORBIDDEN_TEXT = (
    "\ufffd",                     # Unicode replacement character
    "\u951f\u65a4\u62f7",       # common GBK/UTF-8 mojibake marker
    "\u5bb8\u30e7\u25bc\u9358\u71b8\u6783",  # previously damaged README name
)


def ascii_path(path: str) -> bool:
    return all(ord(character) < 128 for character in path)


def looks_like_gbk_utf8_mojibake(line: str) -> bool:
    if sum("\u3400" <= character <= "\u9fff" for character in line) < 2:
        return False
    try:
        repaired = line.encode("gbk").decode("utf-8")
    except (UnicodeEncodeError, UnicodeDecodeError):
        return False
    return repaired != line and any("\u3400" <= character <= "\u9fff" for character in repaired)


def verify_tree(root: Path) -> list[str]:
    errors: list[str] = []
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        if not ascii_path(relative):
            errors.append(f"non-ASCII path: {relative!r}")
        if not path.is_file():
            continue
        if path.suffix.lower() not in TEXT_SUFFIXES and path.name not in TEXT_NAMES:
            continue
        data = path.read_bytes()
        try:
            text = data.decode("utf-8-sig")
        except UnicodeDecodeError as error:
            errors.append(f"not strict UTF-8: {relative}: {error}")
            continue
        for marker in FORBIDDEN_TEXT:
            if marker in text:
                errors.append(f"mojibake marker in text: {relative}: U+{ord(marker[0]):04X}")
        for line_number, line in enumerate(text.splitlines(), 1):
            if looks_like_gbk_utf8_mojibake(line):
                errors.append(f"possible GBK/UTF-8 mojibake: {relative}:{line_number}")
        if path.suffix.lower() in {".bat", ".cmd", ".ps1"}:
            try:
                data.decode("ascii")
            except UnicodeDecodeError:
                errors.append(f"launcher is not ASCII: {relative}")
    return errors


def verify_zip(path: Path) -> list[str]:
    errors: list[str] = []
    with zipfile.ZipFile(path) as archive:
        bad = archive.testzip()
        if bad:
            errors.append(f"ZIP CRC failure: {bad}")
        for info in archive.infolist():
            if not ascii_path(info.filename):
                errors.append(f"non-ASCII ZIP entry: {info.filename!r}")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--zip", dest="zip_path", type=Path)
    args = parser.parse_args()
    errors = verify_tree(args.root.resolve())
    if args.zip_path:
        errors.extend(verify_zip(args.zip_path.resolve()))
    if errors:
        print("RELEASE ENCODING CHECK FAILED")
        for error in errors:
            print(f"- {error}")
        return 1
    if args.zip_path:
        print("PASS: paths are ASCII, text is strict UTF-8, and ZIP CRC is valid.")
    else:
        print("PASS: every release path is ASCII and every text file is strict UTF-8.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
