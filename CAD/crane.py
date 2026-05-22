# %%
from pathlib import Path

from build123d import *
from utils import *
from ocp_vscode import *
from parts import right_feet, left_feet, CRANE_CARRIAGE, PULLEY, ROBOT_CARRIAGE, LEG_CARRIAGE, PROFILE2040, PROFILE2020, ROLLER, SPACER6, PLATE_THICKNESS, STEPPER, M5_round, LEG_PLATE_HEIGHT

CAD_DIR = Path(__file__).resolve().parent
REPO_ROOT = CAD_DIR.parent
SIM_ASSETS = REPO_ROOT / "simulation" / "assets"



# %%
class Crane:
    def __init__(self, rail_length, rail_width, crane_height):
        self.rail_length = rail_length
        self.rail_width = rail_width
        self.crane_height = crane_height
        self.robot_rail_offset = 200
        self.model, self.rails, self.crane_carriage, self.end_effector, self.pulley, self.robot_carriage = self.assembly()


    def rail(self):
        return Pos(0, 0,0)* Rot(X=90)*PROFILE2040.extrude(self.rail_length)
    

    def leg(self):
        carriage = Pos(-PLATE_THICKNESS-SPACER6.height-ROLLER.height/2-ROLLER.rolling_center_offset, 0, LEG_PLATE_HEIGHT/2)*Rot(Y=-90)*LEG_CARRIAGE
        leg = Pos(SPACER6.height+ROLLER.height/2+PLATE_THICKNESS/2+PROFILE2020.height/2, 0, self.crane_height/2)*PROFILE2020.extrude(self.crane_height)
        return Compound(children = [carriage, leg])
    
    def robot_carriage(self):
        return Pos(self.rail_width/2, self.robot_rail_offset,  PLATE_THICKNESS+ SPACER6.height)*Rot(Z=90)*ROBOT_CARRIAGE
         
    

    
    def assembly(self):
        rail1 = Pos(Z=22)*self.rail()
        crane_leg1 = self.leg()
        rail2 = Pos(Z=22)*Pos(X=self.rail_width)*Rot(Z=180)*self.rail()
        robot_rail = Pos(Z=22)*Pos(Y=self.robot_rail_offset)*Pos(self.rail_width/2, 0, PROFILE2040.height/2 + PROFILE2040.width/2)*Rot(Y=90)*PROFILE2040.extrude(self.rail_width+ PROFILE2040.width)
        crane_leg2 = Pos(X=self.rail_width)*Rot(Z=180)*self.leg()
        top_extrusion_length = self.rail_width-2*(SPACER6.height+ROLLER.height/2+PLATE_THICKNESS/2)
        top_extrusion = Pos(SPACER6.height+ROLLER.height/2+PLATE_THICKNESS/2+top_extrusion_length/2, 0, self.crane_height+PROFILE2020.height/2, )*Rot(Y=90)*PROFILE2020.extrude(top_extrusion_length)
        end_effector = Pos(SPACER6.height+ROLLER.height/2+PLATE_THICKNESS/2+top_extrusion_length/2,ROLLER.rolling_center_offset+ROLLER.height/2+PLATE_THICKNESS+SPACER6.height, self.crane_height+23)*Rot(Z=90)*Rot(Y=-90)*Rot(X=180)*CRANE_CARRIAGE
        pulley = Pos(SPACER6.height+ROLLER.height/2+PLATE_THICKNESS/2+top_extrusion_length/2,ROLLER.rolling_center_offset+ROLLER.height/2+PLATE_THICKNESS+SPACER6.height, self.crane_height+23)*Rot(Z=90)*Rot(Y=-90)*Rot(X=180)*PULLEY
        feet_height = right_feet.bounding_box().max.Z - right_feet.bounding_box().min.Z
        feet1 = Pos(self.rail_width, -self.rail_length/2+20, -feet_height/2)*Rot(Z=90)*right_feet
        feet2 = Pos(0, -self.rail_length/2+20, -feet_height/2)*Rot(Z=90)*left_feet
        extrusion_feet = Pos(self.rail_width/2, self.rail_length/2-PROFILE2020.height/2,-PROFILE2040.height/2+22)* Rot(X=90)*Rot(Y=90)*PROFILE2040.extrude(self.rail_width+PROFILE2020.height)
        rails = Compound(children = [rail1, rail2, feet1, feet2, extrusion_feet, robot_rail], label="rails")
        crane_carriage = Compound(children = [crane_leg1, crane_leg2, top_extrusion], label="crane")
        end_effector = Compound(children = [end_effector], label="carriage")
        pulley = Compound(children = [pulley], label="pulley")
        robot_carriage = Pos(Z=22)*self.robot_carriage()
        crane = Compound(children = [rails, crane_carriage, end_effector, pulley, robot_carriage], label="crane")

        return crane, rails, crane_carriage, end_effector, pulley, robot_carriage 
    # -PLATE_THICKNESS-SPACER6.height-ROLLER.height/2-ROLLER.rolling_center_offset
    
    def export_step(self):
        export_step(self.rails, "rails.step")
        export_step(self.model, "crane.step")
        export_step(self.end_effector, "end_effector.step")
        export_step(self.crane_carriage, "crane_carriage.step")

    def export_gltf(self):
        export_gltf(self.rails, "rails.gltf")
        export_gltf(self.model, "crane.gltf")
        export_gltf(self.end_effector, "end_effector.gltf")
        export_gltf(self.crane_carriage, "crane_carriage.gltf")

    def export_stl(self):
        t = Pos(-(self.model.bounding_box().min.X+self.model.bounding_box().max.X)/2, 0, -self.model.bounding_box().min.Z)
        print(-self.model.bounding_box().min.Z)
        export_stl(t*self.rails, str(SIM_ASSETS / "rails.stl"))
        export_stl(t*self.model, str(SIM_ASSETS / "crane.stl"))
        export_stl(t*self.end_effector, str(SIM_ASSETS / "end_effector.stl"))
        export_stl(t*self.crane_carriage, str(SIM_ASSETS / "crane_body.stl"))
        export_stl(t*self.pulley, str(SIM_ASSETS / "pulley.stl"))
        export_stl(t*self.robot_carriage, str(SIM_ASSETS / "robot_carriage.stl"))

c = Crane(980, 910, 671)
c.export_stl()
show(c.model)


# %%
