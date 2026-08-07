"""Generate simplified collision meshes for the force gripper URDF using VHACD.

For each STL in force_gripper_hardware/urdf/meshes, run V-HACD convex
decomposition (official TestVHACD.exe binary in tools/) and write the
resulting hulls into a single multi-object OBJ in meshes/collision/.
Simulators like PyBullet/MuJoCo treat each OBJ object group as its own
convex piece.

Note: the vhacdx Python bindings segfault on this machine, which is why the
standalone binary is used instead.
"""

import subprocess
import tempfile
from pathlib import Path

import trimesh

ROOT = Path(__file__).resolve().parents[1]
VHACD_EXE = ROOT / "tools" / "TestVHACD.exe"
MESH_DIR = ROOT / "urdf" / "meshes"
OUT_DIR = MESH_DIR / "collision"

# V-HACD parameters per mesh: the base is bigger/more complex, the fingers
# are simple pinch surfaces where a few hulls suffice.
# -h max hulls, -r voxel resolution, -v max vertices per hull
VHACD_PARAMS = {
    "base-1": ["-h", "8", "-r", "100000", "-v", "32"],
    "left_figure": ["-h", "4", "-r", "50000", "-v", "16"],
    "right_figure": ["-h", "4", "-r", "50000", "-v", "16"],
}
DEFAULT_PARAMS = ["-h", "8", "-r", "100000", "-v", "32"]


def decompose(stl_path: Path) -> list[trimesh.Trimesh]:
    """Run V-HACD on an STL and return the convex hulls."""
    params = VHACD_PARAMS.get(stl_path.stem, DEFAULT_PARAMS)
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        obj_in = tmp / "input.obj"
        trimesh.load_mesh(stl_path).export(obj_in)
        # TestVHACD writes decomp.obj (all hulls as separate objects) to cwd
        subprocess.run(
            [str(VHACD_EXE), obj_in.name, *params, "-g", "false"],
            cwd=tmp,
            check=True,
            capture_output=True,
        )
        scene = trimesh.load(tmp / "decomp.obj", split_object=True, group_material=False)
    if isinstance(scene, trimesh.Trimesh):
        return [scene]
    return list(scene.geometry.values())


def write_multi_obj(hulls: list[trimesh.Trimesh], path: Path) -> None:
    lines = ["# V-HACD convex decomposition"]
    offset = 1
    for i, h in enumerate(hulls):
        lines.append(f"o convex_{i}")
        lines.extend(f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in h.vertices)
        lines.extend(f"f {a + offset} {b + offset} {c + offset}" for a, b, c in h.faces)
        offset += len(h.vertices)
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    OUT_DIR.mkdir(exist_ok=True)
    for stl in sorted(MESH_DIR.glob("*.stl")):
        src = trimesh.load_mesh(stl)
        hulls = decompose(stl)
        out = OUT_DIR / f"{stl.stem}_collision.obj"
        write_multi_obj(hulls, out)
        n_faces = sum(len(h.faces) for h in hulls)
        print(
            f"{stl.name}: {len(src.faces)} faces -> {len(hulls)} hulls, "
            f"{n_faces} faces total -> {out.relative_to(ROOT)}"
        )


if __name__ == "__main__":
    main()
