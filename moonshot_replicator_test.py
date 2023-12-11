from omni.isaac.kit import SimulationApp
# PathTracing
# RayTracedLighting
CONFIG = {"renderer": "PathTracing", "headless": False, "width": 512, "height": 512, "num_frames": 10, "multi_gpu": True}
simulation_app = SimulationApp(CONFIG)

import carb
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import get_prim_at_path, define_prim
from omni.isaac.core.utils.stage import get_current_stage, add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.objects import DynamicSphere
from omni.isaac.cloner import GridCloner
from omni.isaac.core.robots import Robot
from math import pi
import time

from pathlib import Path
path_str = str(Path(__file__).parent)
USD_PATH = path_str+"/scenes/moonshot_replicator_lab.usd" 
output_directory = r"E:\output\_out_test"                # The folder path to save the synthetic dataset

# create the world
world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene", backend="numpy")
#world.scene.add_default_ground_plane()

# set up grid cloner
cloner = GridCloner(spacing=1.5)
cloner.define_base_env("/World/envs")
define_prim("/World/envs/env_0")

# set up the first environment
DynamicSphere(prim_path="/World/envs/env_0/object", radius=0.1, position=np.array([0.75, 0.0, 0.2]))
#add_reference_to_stage(
#    usd_path=get_assets_root_path() + "/Isaac/Robots/Franka/franka.usd", prim_path="/World/envs/env_0/franka"
#)
add_reference_to_stage(usd_path=USD_PATH , prim_path="/World")
z_ = 8.0
cobotta1 = world.scene.add(Robot(prim_path="/World/envs/env_0/root/Cobotta_robot_1", name="cobotta1", position=np.array([0.0, 0.0, z_])))
cobotta2 = world.scene.add(Robot(prim_path="/World/envs/env_0/root/Cobotta_robot_2", name="cobotta2", position=np.array([0.0, 0.0, z_])))
denso_1 =  world.scene.add(Robot(prim_path="/World/envs/env_0/root/Denso_robot_1",   name="denso1",   position=np.array([0.0, 0.0, z_])))
denso_2 =  world.scene.add(Robot(prim_path="/World/envs/env_0/root/Denso_robot_2",   name="denso2",   position=np.array([0.0, 0.0, z_])))

# clone environments
num_envs = 1
prim_paths = cloner.generate_paths("/World/envs/env", num_envs)
env_pos = cloner.clone(source_prim_path="/World/envs/env_0", prim_paths=prim_paths)

# creates the views and set up world
object_view = RigidPrimView(prim_paths_expr="/World/envs/*/object", name="object_view")
cobotta1_view = ArticulationView(prim_paths_expr="/World/envs/env_0/root/Cobotta_robot_1", name="cobotta1_view")
cobotta2_view = ArticulationView(prim_paths_expr="/World/envs/env_0/root/Cobotta_robot_2", name="cobotta2_view")
denso1_view =   ArticulationView(prim_paths_expr="/World/envs/env_0/root/Denso_robot_1",   name="denso1_view")
denso2_view =   ArticulationView(prim_paths_expr="/World/envs/env_0/root/Denso_robot_2",   name="denso2_view")

world.scene.add(object_view)
world.scene.add(cobotta1_view)
world.scene.add(cobotta2_view)
world.scene.add(denso1_view)
world.scene.add(denso2_view)

world.reset()

num_dof_cobotta = cobotta1_view.num_dof
num_dof_denso = denso1_view.num_dof

# set up randomization with omni.replicator.isaac, imported as dr
import omni.replicator.isaac as dr
import omni.replicator.core as rep

dr.physics_view.register_simulation_context(world)
dr.physics_view.register_rigid_prim_view(object_view)
dr.physics_view.register_articulation_view(cobotta1_view)
dr.physics_view.register_articulation_view(cobotta2_view)
dr.physics_view.register_articulation_view(denso1_view)
dr.physics_view.register_articulation_view(denso2_view)

frames = 10000
camera = rep.create.camera(position=(6.435, -0.31, 13.654), clipping_range=(0.01, 10000.0), focus_distance=18)
#rep.set_global_seed(23)
rep.settings.set_render_pathtraced(samples_per_pixel=512)
rep.settings.carb_settings("/omni/replicator/RTSubframes", 5)
render_product = rep.create.render_product(camera, (512, 512)) #(512, 512)


cobotta2_rjoint6_lb = -25*(pi/180)
cobotta2_rjoint6_ub = -15*(pi/180)

cobotta2_rjoint5_lb = -105*(pi/180)  
cobotta2_rjoint5_ub = -85*(pi/180) 

cobotta2_rjoint4_lb = 50*(pi/180)
cobotta2_rjoint4_ub = 90*(pi/180)

cobotta2_rjoint3_lb = 65*(pi/180) - 90*(pi/180)
cobotta2_rjoint3_ub = 68*(pi/180) - 90*(pi/180)

cobotta2_rjoint1_lb = -11*(pi/180)
cobotta2_rjoint1_ub = -10*(pi/180)

cobotta2_rjoint2_lb = 22*(pi/180)
cobotta2_rjoint2_ub = 28*(pi/180)


q_cobotta2_lower = [-0.1, 0,  cobotta2_rjoint1_lb, cobotta2_rjoint2_lb, cobotta2_rjoint3_lb,    cobotta2_rjoint4_lb, cobotta2_rjoint5_lb, cobotta2_rjoint6_lb]
q_cobotta2_upper = [ 0.1, 1,  cobotta2_rjoint1_ub, cobotta2_rjoint2_ub, cobotta2_rjoint3_ub,    cobotta2_rjoint4_ub, cobotta2_rjoint5_ub, cobotta2_rjoint6_ub]


cobotta1_rjoint1_lb = 10*(pi/180)
cobotta1_rjoint1_ub = 11*(pi/180)

cobotta1_rjoint5_lb = -25*(pi/180)  
cobotta1_rjoint5_ub = -18*(pi/180) 


cobotta1_rjoint6_lb = 15*(pi/180)
cobotta1_rjoint6_ub = 25*(pi/180)

q_cobotta1_lower = [-0.1, 0,  cobotta1_rjoint1_lb, cobotta2_rjoint2_lb, cobotta2_rjoint3_lb,    cobotta2_rjoint4_lb, cobotta1_rjoint5_lb, cobotta1_rjoint6_lb]
q_cobotta1_upper = [ 0.1, 1,  cobotta1_rjoint1_ub, cobotta2_rjoint2_ub, cobotta2_rjoint3_ub,    cobotta2_rjoint4_ub, cobotta1_rjoint5_ub, cobotta1_rjoint6_ub]



#qD_lower = [-0.25, 0, -1, d4_lower,   d5_lower,    -1, -1, -1, -1, 0,0,0,0]
#qD_upper = [ 0.25, 2,  1, d4_upper,   d5_upper,     1,  1,  1,  1, 0,0,0,0]

k = 3
qC_dot_lower = [-k, -k, -k, -k,   -k,    -k, -k, -k]
qC_dot_upper = [ k,  k,  k,  k,    k,     k,  k,  k]
qD_dot_lower = [-k, -k, -k, -k,   -k,    -k, -k, -k, -k, 0,0,0,0]
qD_dot_upper = [ k,  k,  k,  k,    k,     k,  k,  k,  k, 0,0,0,0]


with dr.trigger.on_rl_frame(num_envs=num_envs):   #num_frames=CONFIG["num_frames"]   num_envs=num_envs
    """
    with dr.gate.on_interval(interval=20):
        dr.physics_view.randomize_simulation_context(
            operation="scaling", gravity=rep.distribution.uniform((1, 1, 0.0), (1, 1, 2.0))
        )
    with dr.gate.on_interval(interval=50):
        dr.physics_view.randomize_rigid_prim_view(
            view_name=object_view.name, operation="direct", force=rep.distribution.uniform((0, 0, 2.5), (0, 0, 5.0))
        )
    ######
        
    
    with dr.gate.on_interval(interval=10):      
        dr.physics_view.randomize_articulation_view(
            view_name=cobotta1_view.name,
            operation="direct",
            #joint_velocities=rep.distribution.uniform(tuple([-2] * num_dof_cobotta), tuple([2] * num_dof_cobotta)),
            #joint_velocities=rep.distribution.uniform(qC_lower, qC_upper),
            #joint_velocities=rep.distribution.uniform(qC_lower, qC_upper),
            joint_velocities=rep.distribution.uniform(qC_dot_lower, qC_dot_upper),
        )
        
        dr.physics_view.randomize_articulation_view(
            view_name=cobotta2_view.name,
            operation="direct",
            #joint_velocities=rep.distribution.uniform(qC_lower, qC_upper),
            joint_velocities=rep.distribution.uniform(qC_dot_lower, qC_dot_upper),
        )
        
        dr.physics_view.randomize_articulation_view(
            view_name=denso1_view.name,
            operation="direct",
            #joint_velocities=rep.distribution.uniform(qD_lower, qD_upper),
            joint_velocities=rep.distribution.uniform(qD_dot_lower, qD_dot_upper),
        )
        
        dr.physics_view.randomize_articulation_view(
            view_name=denso2_view.name,
            operation="direct",
            #joint_velocities=rep.distribution.uniform(qD_lower, qD_upper),
            joint_velocities=rep.distribution.uniform(qD_dot_lower, qD_dot_upper),
        )
        """
    with dr.gate.on_env_reset():
            dr.physics_view.randomize_rigid_prim_view(
                view_name=object_view.name,
                operation="additive",
                position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
                velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
            
            dr.physics_view.randomize_articulation_view(
                view_name=cobotta1_view.name,
                operation="additive",
                joint_positions=rep.distribution.uniform(q_cobotta1_lower,q_cobotta1_upper),
                position=rep.distribution.normal((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            )
            
            
            dr.physics_view.randomize_articulation_view(
                view_name=cobotta2_view.name,
                operation="additive",
                joint_positions=rep.distribution.uniform(q_cobotta2_lower, q_cobotta2_upper),
                position=rep.distribution.normal((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            )
            """
            dr.physics_view.randomize_articulation_view(
                view_name=denso1_view.name,
                operation="additive",
                joint_positions=rep.distribution.uniform(qD_lower, qD_upper),
                position=rep.distribution.normal((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            )

            dr.physics_view.randomize_articulation_view(
                view_name=denso2_view.name,
                operation="additive",
                joint_positions=rep.distribution.uniform(qD_lower, qD_upper),
                position=rep.distribution.normal((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ) 
           """   
             
  
#camera = rep.create.camera(position=(6.435, -0.31, 13.654), clipping_range=(0.01, 10000.0)) 

with rep.trigger.on_frame(num_frames=CONFIG["num_frames"] , interval=1,  rt_subframes=5):   
        with camera:
            rep.modify.pose(   
                    position=rep.distribution.uniform((6, -0.4, 12), (7, -0.2, 16)),
                    look_at="/World/Apple",
                    #scale=rep.distribution.uniform(0.1, 2)
                )
            
        
            

# Initialize and attach writer
writer = rep.WriterRegistry.get("BasicWriter")  #rep.WriterRegistry.get("YCBVideoWriter") #
#output_directory = os.getcwd() + "/_output_headless"
#output_directory = r"C:\Users\juanjqo\Desktop\output"

print("Outputting data to ", output_directory)     
writer.initialize(
        output_dir=output_directory,
        rgb=True,
        #bounding_box_2d_tight=True,
        #bounding_box_2d_loose=True,
        semantic_segmentation=True,
        #instance_segmentation=True,
        #distance_to_camera=True,
        #distance_to_image_plane=True,
        #bounding_box_3d=True,
        #occlusion=True,
        #normals=True,
        #motion_vectors=True,
    )
    
    #writer.initialize( output_dir=r"C:\Users\juanjqo\Desktop\output", rgb=True,   bounding_box_2d_tight=True)
writer.attach([render_product])   

  

frame_idx = 0
while simulation_app.is_running():
    dr.physics_view.step_randomization(np.arange(num_envs))
    print(f"frame_idx={frame_idx}")

    # trigger replicator randomizers and data collection
    rep.orchestrator.step()
    world.step(render=True)
    frame_idx += 1
    if frame_idx >= CONFIG["num_frames"] :
        simulation_app.close()

        

