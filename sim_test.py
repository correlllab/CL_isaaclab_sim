from isaacsim import SimulationApp
import math
import os

simulation_app = SimulationApp({
   "headless": True,
   "renderer": "RayTracedLighting",
   "useExtension": True,
})


import omni.kit.app

kit = omni.kit.app.get_app()
kit.get_extension_manager().set_extension_enabled_immediate("omni.replicator.core", True)
import omni.replicator.core as rep

def main():
   OUTPUT_DIR = "/workspace/reach_ws/outputs"
   os.makedirs(OUTPUT_DIR, exist_ok=True)

   # Create a plane
   rep.create.plane(scale=(1, 1, 1), position=(0, 0, 0))

   # Create cube
   cube = rep.create.cube(position=(-0.2, 0, 0.3), scale=(0.5, 0.5, 0.5))

   # Create sphere
   sphere = rep.create.sphere(position=(0.2, 0, 0.3), scale=(0.2, 0.2, 0.2))

   # Create a light, I don't have a texture local... need to update with: 
   # texture="omniverse://localhost/NVIDIA/Assets/Skies/CloudySky.exr"
   rep.create.light(
      light_type="dome",
      intensity=1000
   )

   # Create the camera and be sure the cube and sphere are in sight    
   camera = rep.create.camera(
       position=(0, -1.5, 0.5),  # Pull back and raise the camera
       look_at=(0, 0, 0.3),      # Aim directly at the cube/sphere
       clipping_range=(0.01, 1000.0)
   )

   # Render product
   rep.orchestrator.step(rt_subframes=4)
   render_product = rep.create.render_product(camera, (640, 480))

   # Writer setup
   writer = rep.WriterRegistry.get("BasicWriter")
   writer.initialize(output_dir=OUTPUT_DIR, rgb=True)
   writer.attach([render_product])

   # Make sure simulation_app.update() is called at least once before rendering begins. 
   # This helps initialize the stage and extensions properly.
   simulation_app.update()

   # Let it warm up and render
   rep.orchestrator.run()
   for _ in range(10):
       rep.orchestrator.step()
       absolute_path = os.path.abspath(OUTPUT_DIR)
       print(absolute_path)
       print(f"Image saved to {OUTPUT_DIR}/replicator_0/RenderProduct_0/rgb_00{_}.png")

   writer.detach()

   print(f"Render complete")

   simulation_app.close()

if __name__ == "__main__":
   main()



