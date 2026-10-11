# PyInstaller spec for IDPS.app (macOS, Apple Silicon or Intel - build on the
# architecture you want to ship). Run via packaging/macos/build_app.sh.
from pathlib import Path

root = Path(SPECPATH).resolve().parents[1]

a = Analysis(
    [str(root / "packaging" / "macos" / "launch.py")],
    pathex=[str(root)],
    datas=[(str(root / "examples" / "sites"), "examples/sites"), (str(root / "docs"), "docs")],
    hiddenimports=["matplotlib.backends.backend_agg"],
    excludes=["tkinter", "PySide6.QtWebEngineCore", "PySide6.Qt3DCore"],
)
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, [], exclude_binaries=True, name="IDPS", console=False,
          argv_emulation=False, target_arch=None)
coll = COLLECT(exe, a.binaries, a.datas, name="IDPS")
app = BUNDLE(
    coll,
    name="IDPS.app",
    bundle_identifier="local.idps.dashboard",
    info_plist={
        "CFBundleDisplayName": "IDPS",
        "CFBundleShortVersionString": "0.1.0",
        "NSHighResolutionCapable": True,                 # Retina rendering
        "NSRequiresAquaSystemAppearance": False,         # follow macOS dark mode
        "LSMinimumSystemVersion": "12.0",
        "CFBundleDocumentTypes": [{
            "CFBundleTypeName": "IDPS venue",
            "CFBundleTypeRole": "Editor",
            "LSHandlerRank": "Alternate",
            "CFBundleTypeExtensions": ["yaml", "yml"],
        }],
    },
)
