"""Simple test script to verify drone spawning works"""

from omni.isaac.orbit.app import AppLauncher

# Create minimal app launcher
app_launcher = AppLauncher({"headless": False})
simulation_app = app_launcher.app

# Now import Isaac Sim modules
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.isaac.core.utils.stage as stage_utils

# Create world
world = World()

print("[INFO] Spawning simple drone...")

# List of possible drone asset paths to try
drone_assets = [
    "/Isaac/Robots/Crazyflie/cf2x.usd",
    "/Isaac/Robots/Quadcopter/quadcopter.usd", 
    "/Isaac/Props/Blocks/DexCube/dex_cube_instanceable.usd",  # Fallback cube
]

drone_spawned = False

for asset_rel_path in drone_assets:
    try:
        drone_path = "/World/Drone"
        
        # Get Isaac Sim assets root
        from omni.isaac.core.utils.nucleus import get_assets_root_path
        assets_root = get_assets_root_path()
        
        if assets_root is None:
            print("[WARN] Could not get assets root path, using local path")
            asset_path = asset_rel_path
        else:
            asset_path = assets_root + asset_rel_path
        
        print(f"[INFO] Trying asset: {asset_path}")
        
        add_reference_to_stage(asset_path, drone_path)
        
        # Check if it worked
        prim = prim_utils.get_prim_at_path(drone_path)
        if prim.IsValid():
            print(f"[SUCCESS] Drone spawned from {asset_rel_path}")
            
            # Move it up
            prim_utils.set_prim_attribute_value(drone_path, "xformOp:translate", (0.0, 0.0, 1.5))
            
            drone_spawned = True
            break
        else:
            print(f"[WARN] Prim not valid, trying next asset...")
            stage_utils.clear_stage()
            world = World()
            
    except Exception as e:
        print(f"[WARN] Failed to load {asset_rel_path}: {e}")
        stage_utils.clear_stage()
        world = World()
        continue

if not drone_spawned:
    print("[INFO] All asset attempts failed, creating simple cube placeholder...")
    
    # Create a simple cube using USD directly
    from pxr import UsdGeom, Gf
    
    stage = omni.usd.get_context().get_stage()
    drone_prim = stage.DefinePrim("/World/SimpleDrone", "Xform")
    
    # Add cube geometry
    cube = UsdGeom.Cube.Define(stage, "/World/SimpleDrone/body")
    cube.GetSizeAttr().Set(0.3)
    cube.GetDisplayColorAttr().Set([(0.2, 0.5, 1.0)])  # Blue
    
    # Set position
    xform = UsdGeom.Xformable(drone_prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 1.5))
    
    print("[SUCCESS] Simple cube drone created")

print("[INFO] Adding ground plane for reference...")
from omni.isaac.core.objects import GroundPlane
import numpy as np
ground = GroundPlane(prim_path="/World/ground", size=10.0, color=np.array([0.5, 0.5, 0.5]))

print("[INFO] Adding lights to scene...")
from pxr import UsdLux
import omni.usd
stage = omni.usd.get_context().get_stage()
distant_light = UsdLux.DistantLight.Define(stage, "/World/defaultLight")
distant_light.CreateIntensityAttr(1000)

print("[INFO] Checking drone in scene...")
drone_prim = prim_utils.get_prim_at_path(drone_path)
if drone_prim and drone_prim.IsValid():
    print(f"[SUCCESS] Drone prim is valid at {drone_path}")
    print(f"[INFO] Drone type: {drone_prim.GetTypeName()}")
    # Check if it has a transform
    from pxr import UsdGeom
    xformable = UsdGeom.Xformable(drone_prim)
    if xformable:
        print(f"[INFO] Drone is transformable")
else:
    print(f"[ERROR] Drone prim is NOT valid!")

print("[INFO] Setting up viewport camera to view drone...")
try:
    from omni.isaac.core.utils.viewports import set_camera_view
    # Position camera to look at the drone from closer
    set_camera_view(eye=[2.0, 2.0, 1.5], target=[0.0, 0.0, 1.0], camera_prim_path="/OmniverseKit_Persp")
    print("[SUCCESS] Camera positioned to view drone")
except Exception as e:
    print(f"[WARN] Could not set camera view: {e}")

print("[INFO] Resetting world to initialize physics...")
world.reset()

print("[INFO] Running simulation - Press Ctrl+C to exit")
print("[INFO] Look for:")
print("  - Gray ground plane (10x10m)")
print("  - Drone at position (0, 0, 1.5m height)")
print("[INFO] Camera controls:")
print("  - Mouse wheel: Zoom in/out")
print("  - Middle mouse: Pan camera")
print("  - Alt+Left mouse: Orbit view")

for i in range(1000):
    world.step(render=True)
    if i % 100 == 0:
        # Check drone position
        try:
            pos = prim_utils.get_prim_property(drone_path, "xformOp:translate")
            print(f"[INFO] Step {i} - Drone position: {pos}")
        except:
            print(f"[INFO] Step {i} - Could not get drone position")

print("[INFO] Test complete")
simulation_app.close()

