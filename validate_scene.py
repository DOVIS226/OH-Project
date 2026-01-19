#!/usr/bin/env python3
"""Simple script to validate the MuJoCo scene XML"""

import sys
try:
    import mujoco
    print("MuJoCo imported successfully")

    try:
        model = mujoco.MjModel.from_xml_path('bedroom_scene.xml')
        print("✓ Scene loaded successfully!")
        print(f"  Bodies: {model.nbody}")
        print(f"  Geoms: {model.ngeom}")
        print(f"  Cameras: {model.ncam}")
        sys.exit(0)
    except Exception as e:
        print(f"✗ Error loading scene: {e}")
        sys.exit(1)

except ImportError as e:
    print(f"✗ MuJoCo not installed: {e}")
    sys.exit(1)
