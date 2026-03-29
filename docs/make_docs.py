#!/usr/bin/env python3
#
# Build MkDocs HTML documentation into destination_root/site/
#
# Usage:
#   make_docs.py <src_root> <destination_root>
#
# Example (from repo root):
#   python docs/make_docs.py . build/docs
#
import subprocess
import sys
from pathlib import Path


def main() -> None:
    if len(sys.argv) < 3:
        print("make_docs.py <src_root> <destination_root>", file=sys.stderr)
        sys.exit(1)
    src_root = Path(sys.argv[1]).resolve()
    dest_root = Path(sys.argv[2]).resolve()
    site_dir = dest_root / "site"
    site_dir.parent.mkdir(parents=True, exist_ok=True)
    mkdocs_yml = src_root / "docs" / "mkdocs.yml"
    if not mkdocs_yml.is_file():
        print(f"Missing {mkdocs_yml}", file=sys.stderr)
        sys.exit(1)
    subprocess.check_call(
        [
            sys.executable,
            "-m",
            "mkdocs",
            "build",
            "--strict",
            "-f",
            str(mkdocs_yml),
            "-d",
            str(site_dir),
        ],
        cwd=str(src_root),
    )


if __name__ == "__main__":
    main()
