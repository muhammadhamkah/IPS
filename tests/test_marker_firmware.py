"""The marker firmware's code generator (marker/main/markercode.c) blinks exactly the code the engine
decodes (idps/protocol/markercode.py), for every ID."""
import shutil
import subprocess
from pathlib import Path

import pytest

from idps.protocol import markercode

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "marker" / "main"

MAIN = r"""
#include <stdio.h>
#include "markercode.h"
int main(void) {
    uint8_t c[MC_CHIPS];
    for (int id = 0; id <= MC_ID_MAX; id++) {
        mc_chips(id, c);
        for (int i = 0; i < MC_CHIPS; i++) putchar('0' + c[i]);
        putchar('\n');
    }
    return mc_chips(256, c) == -1 && mc_chips(-1, c) == -1 ? 0 : 1;
}
"""


def test_firmware_code_matches_the_engine_for_every_id(tmp_path):
    cc = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if cc is None:
        pytest.skip("needs a C compiler")
    (tmp_path / "t.c").write_text(MAIN)
    exe = tmp_path / "t"
    subprocess.run([cc, "-std=c99", "-Wall", "-Werror", "-I", str(SRC), str(tmp_path / "t.c"), str(SRC / "markercode.c"),
                    "-o", str(exe)], check=True)
    lines = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.split()
    assert len(lines) == 256
    for marker_id, line in enumerate(lines):
        want = "".join(str(int(v)) for v in markercode.chips(marker_id))
        assert line == want, marker_id
        assert markercode.decode_chips([int(ch) for ch in line]) == marker_id
