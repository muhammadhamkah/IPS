#!/usr/bin/env bash
# Build dist/IDPS.app on a Mac. Requires Python 3.11+ (python.org or Homebrew).
#   bash packaging/macos/build_app.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
python3 -m venv .venv-macapp
source .venv-macapp/bin/activate
pip install --upgrade pip
pip install -e '.[mac-app]'
pyinstaller --noconfirm --clean --distpath dist --workpath build/pyinstaller packaging/macos/idps.spec
echo
echo "Built dist/IDPS.app - drag it to /Applications."
echo "The app is unsigned: on first launch, right-click IDPS.app > Open (or allow it in"
echo "System Settings > Privacy & Security)."
