"""List available XML/MJCF files in this repository."""

from stage1_common import ROOT, XML_PATH

excluded_dirs = {".venv", ".ruff_cache", "__pycache__"}

for path in sorted(ROOT.rglob("*.xml")):
    if any(part in excluded_dirs for part in path.relative_to(ROOT).parts):
        continue
    print(path)

print()
print("Selected XML_PATH =", XML_PATH)
