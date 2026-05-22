# %%
from pathlib import Path

from build123d import *
from utils import *
from ocp_vscode import *
from bd_warehouse.fastener import SetScrew, ClearanceHole, HeatSetNut, InsertHole, HexNut, HexHeadScrew
from bd_warehouse.bearing import SingleRowDeepGrooveBallBearing, PressFitHole

CAD_DIR = Path(__file__).resolve().parent
INPUT_MODELS = CAD_DIR / "input_models"

cap_screw_m3 = SetScrew(size="M3-0.5", length=16)
cap_screw_m4 = SetScrew(size="M4-0.7", length=16)
cap_screw_m6 = SetScrew(size="M6-1", length=16)
cap_screw_m8 = SetScrew(size="M8-1.25", length=16)
heat_set_m5 = HeatSetNut(size="M5-0.8-Standard", simple=True)
PROFILE2040 = AluminiumExtrustion(INPUT_MODELS / "Motedis Profile 20x40 B-Type slot 6.stp", source="https://www.motedis.be/fr/Profile-aluminium-20x40-4N-Type-I-rainure-5")
PROFILE2020 = AluminiumExtrustion(INPUT_MODELS / "Motedis Profile 20x20 B-Type slot 6.stp", source="https://www.motedis.be/en/Aluminium-Profile-20x20-I-Typ-slot-5?added_to_cart=1&show_cart=1&products_id=999991076")

ROLLER = Roller(INPUT_MODELS / "Motedis_roller_29mm_slot6.STEP", 23, 1.6)
SPACER3= Spacer(3, 8, 6)
SPACER6= Spacer(6, 8, 6)
STEPPER = StepperMotor(step_file=INPUT_MODELS / "Nema17.step", case_height=42.5)

# %%
#### ENDSTOP HOLDER

r_dim = 26.0
support_width = 5.0
base_height = 20.0
support_height = 11.0
support_offset = 3.0
extrusion_rail_width = 6.0
r =Box(r_dim , r_dim , base_height, align=[Align.CENTER, Align.CENTER, Align.MIN])
l = Pos(r_dim/2-support_width/2, support_offset, base_height)*Box(support_width, r_dim, support_height, align=[Align.CENTER, Align.CENTER, Align.MIN])
l2 = Pos(0, r_dim/2, base_height/2)*(Box(r_dim, 3,  extrusion_rail_width, align=[Align.CENTER, Align.MIN ,Align.CENTER ]) - Box(r_dim/2, 3,  extrusion_rail_width, align=[Align.CENTER, Align.MIN ,Align.CENTER ]))

l = l -Pos(0,r_dim/2-3.5+support_offset, base_height+support_height/2)*Rot(Y=-90)*ClearanceHole(fastener=cap_screw_m4, depth=100)
l = l -Pos(0, -r_dim/2+3.5+support_offset, base_height+support_height/2)*Rot(Y=-90)*ClearanceHole(fastener=cap_screw_m4, depth=100)
f = extrude(Polygon((0.0, 0.0), (r_dim, 0.0), (0.0, r_dim)), base_height)
r = r-f +l + l2
r = r -Pos(0,3.5, base_height/2)*Pos(Y=20)*Rot(X=-90)*ClearanceHole(fastener=cap_screw_m4, depth=100)
r = r-Pos(Z=base_height/2)*Box(14, 14, 10)
endstop_holder = Rot(Y=90)*Compound(children=[r])
endstop_holder_mirrored = mirror(endstop_holder , Plane.ZX)
#export_stl(endstop_holder, "parts/endstop_holder.stl")
show(endstop_holder, l)


# %%
#### Extrusion feets
b = Box(60, 40, 80, align=[Align.CENTER, Align.CENTER, Align.MIN])
e = Pos(-20, 0, 40)*Box(100, 20, 40, align=[Align.MIN, Align.CENTER, Align.MIN])
p = b-e


# Bottom screws
p = p - Pos(0, 0, 0)*Rot(X=180)*InsertHole(heat_set_m5, depth=10)
p = p - Pos(20, 10, 0)*Rot(X=180)*InsertHole(heat_set_m5, depth=10)
p = p - Pos(-20, 10, 0)*Rot(X=180)*InsertHole(heat_set_m5, depth=10)
p = p - Pos(20, -10, 0)*Rot(X=180)*InsertHole(heat_set_m5, depth=10)

# Side screw M6
p = p - Pos(-30 ,0, 50)*Rot(Y=-90)*ClearanceHole(fastener=cap_screw_m6, depth=100)
p = p - Pos(-30 ,0, 70)*Rot(Y=-90)*ClearanceHole(fastener=cap_screw_m6, depth=100)
t = Pos(Y=-37.5)* extrude(Polygon((0, 0), (0,-35 ), (-60, 0)), -80)
p = p+t
p = fillet(p.edges().group_by(Axis.X)[0][0],1)#this is weird
right_feet = fillet(p.edges().group_by(Axis.Z)[-1][0:6], 2)

ta = Pos(40, 20 ,25)*Torus(50, 10)
right_feet = right_feet - ta
left_feet = mirror(right_feet, Plane.ZX)
#export_stl(right_feet, "parts/right_feet.stl")
#export_stl(left_feet, "parts/left_feet.stl")
show(right_feet, left_feet)



# %%
#### Pulley
base_od = 60
base_id = 50
base_wall_thickness = 1.5
base_height = 8
motor_shaft_lenght = 25
shaft_od = 20
shaft_id = 5
shaft_height = 7
ALIGN = [Align.CENTER, Align.CENTER, Align.MIN]
shaft =Cylinder(base_od/2, base_wall_thickness, align=ALIGN) + Pos(0, 0, base_wall_thickness)*Cylinder(base_id/2, base_height, align=ALIGN) + Pos(0, 0, base_height+base_wall_thickness)*Cylinder(base_od/2, base_wall_thickness, align=ALIGN)
shaft = chamfer(shaft.edges()[3], length=4.99, length2=2)
shaft = chamfer(shaft.edges()[7], length=2, length2=4.99)
#t = Pos(base_id/2, 0, base_wall_thickness)*Polygon((0, 0, 0), ((base_od-base_id), 0, 0), (0, 0, 2))

screw = HexHeadScrew(size="M8-1.25", length=80)
screw = Pos(Z=screw.head_height-0.5)*Rot(Y=180)*scale(screw, (1.015, 1.015, 1.015))
shaft =  shaft - screw
shaft = shaft -Cylinder(4.015,50)
belt_pulley = import_stl(str(INPUT_MODELS / "gt2-80tooth-8mm-shaft-6mm-belt.stl"))
belt_pulley = Pos(Z=base_height+2*base_wall_thickness+belt_pulley.bounding_box().size.Z)* Rot(Y=180)*belt_pulley
r = Pos(0, base_id/2-10, 2+base_wall_thickness)*Box(2, 20, 4, align=ALIGN)
shaft = shaft-r
pulley = Compound(children=[shaft, belt_pulley]) 

export_stl(pulley, "parts/pulley.stl")
show(pulley)


#export_stl(shaft, 'shaft.stl')

# %%
#### LEG PLATE CARRIAGE

c_0 = Circle(4.9)
c_1 = Circle(2.5)
M5 = loft([c_0, Pos(Z=-3)*c_1, Pos(Z=-28)*c_1], ruled=True)
M5_round = Cylinder(2.5, 25, align=[Align.CENTER, Align.CENTER, Align.MAX])
M6_round = Cylinder(3, 50, align=[Align.CENTER, Align.CENTER, Align.MAX])

PLATE_THICKNESS = 5
STEPPER_DIMENSIONS = (42, 42, 28)
STEPPER_FIXING_HOLES_DISTANCE = 21 - 5.5
STEPPER_SHAFT_CIRCLE_RADIUS =12

#PULLEY = Cylinder(8, 16, align=[Align.CENTER, Align.CENTER, Align.MAX])

LEG_PLATE_HEIGHT = 115
LEG_PLATE_WIDTH = 70

CRANE_PLATE_HEIGHT = 80
CRANE_PLATE_WIDTH = 70

STEPPER_CENTER = Location((30, 0, 0))


#### LEG PLATE CARRIAGE
leg_plate = Box(LEG_PLATE_HEIGHT,LEG_PLATE_WIDTH , PLATE_THICKNESS)
leg_plate = fillet(leg_plate.edges().filter_by(Axis.Z), radius=7)


leg_plate1 = stepper_mount(leg_plate, STEPPER, STEPPER_CENTER, STEPPER_SHAFT_CIRCLE_RADIUS, STEPPER_FIXING_HOLES_DISTANCE)
ROLLERS_CENTER = Location((-15, 0, leg_plate.bounding_box().max.Z))

leg_plate1, rollers = plate_to_cariage(leg_plate1, PROFILE2040, 52, ROLLERS_CENTER, M6_round, ROLLER, SPACER6)
leg_plate1 -= Pos(LEG_PLATE_HEIGHT/2-8, LEG_PLATE_WIDTH/2-8)* Cylinder(2, 20)
leg_plate1 -= Pos(LEG_PLATE_HEIGHT/2-8, -LEG_PLATE_WIDTH/2+8)* Cylinder(2, 20)

leg_plate2 = leg_plate 
leg_plate2= leg_plate2 + Pos(-40, LEG_PLATE_WIDTH/2, PLATE_THICKNESS/2-0.5)*Rot(Y=90)*Box(1, 10, 20)
leg_plate2 = leg_plate2 + Pos(-40, -LEG_PLATE_WIDTH/2, PLATE_THICKNESS/2-0.5)*Rot(Y=90)*Box(1, 10, 20)

screws =[]
for i in range(3):
    leg_plate2 = leg_plate2-(Pos((i+1)*LEG_PLATE_HEIGHT/4-LEG_PLATE_HEIGHT/2, 0, -PLATE_THICKNESS/2)*Rot(Y=180)*M5_round)
leg_plate2 = Pos(Z = -2*SPACER6.height-ROLLER.height-PLATE_THICKNESS)*Rot(X=180)*leg_plate2
leg_plate2, _ =  plate_to_cariage(leg_plate2, PROFILE2040, 52, ROLLERS_CENTER, M6_round, ROLLER, SPACER6)
export_stl(leg_plate1, "leg_plate1.stl")
export_stl(leg_plate2, "leg_plate2.stl")
stepper = STEPPER_CENTER*Pos(Z=STEPPER.case_height)*Rot(X=180)*STEPPER.model
LEG_CARRIAGE = Pos(Z=-PLATE_THICKNESS/2)*Compound(label="carriage", children = [leg_plate1, leg_plate2, *rollers, stepper])
show(LEG_CARRIAGE)
# %%
#### Endeffector CARRIAGE 


SPACER6= Spacer(6, 8, 6)
BEARING =  SingleRowDeepGrooveBallBearing(size="M8-22-7")
M6_round = Cylinder(3, 50, align=[Align.CENTER, Align.CENTER, Align.MAX])
M11 = Cylinder(5.5, 50, align=[Align.CENTER, Align.CENTER, Align.MAX])  
PLATE_THICKNESS = 5
STEPPER_DIMENSIONS = (42, 42, 28)
STEPPER_FIXING_HOLES_DISTANCE = 21 - 5.5
STEPPER_SHAFT_CIRCLE_RADIUS =12

CRANE_MOTOR_PLATE_HEIGHT = 100
CRANE_SHAFT_PLATE_HEIGHT = 115
CRANE_PLATE_WIDTH = 70
PLATE_THICKNESS = 5


bearings = []
crane_plate1 = Box(CRANE_MOTOR_PLATE_HEIGHT,CRANE_PLATE_WIDTH , PLATE_THICKNESS)
crane_plate1 = fillet(crane_plate1.edges().filter_by(Axis.Z), radius=7)
crane_plate2 = Box(CRANE_SHAFT_PLATE_HEIGHT,CRANE_PLATE_WIDTH , PLATE_THICKNESS)
crane_plate2 = fillet(crane_plate2.edges().filter_by(Axis.Z), radius=7)
crane_plate1 = crane_plate1+Pos(-28, 0,0)* Pos(Z=3)*Cylinder(14, 2)
crane_plate1 = crane_plate1- Pos(-28, 0, -PLATE_THICKNESS/2)* Rot(X=180)*PressFitHole(bearing=BEARING, depth=7) 



crane_plate2 = Pos(X=-40)*crane_plate2
crane_plate2 = crane_plate2+Pos(-28, 0,0)* Pos(Z=3)*Cylinder(14, 2)
crane_plate2 = crane_plate2- Pos(-28, 0, -PLATE_THICKNESS/2)* Rot(X=180)*PressFitHole(bearing=BEARING, depth=7) 

crane_plate2 = crane_plate2- Pos(-28, 0, -PLATE_THICKNESS/2-6)* Rot(X=180)* ClearanceHole(fastener=SetScrew(size="M8-1.25", length=20), depth=20, fit="Loose")
bearings.append(Pos(-28, 0, -5)*BEARING)
#crane_plate2 = fillet(crane_plate2.edges().group_by(Axis.Z)[3], radius=1)


bearings.append(Pos(-28, 0, -2*SPACER6.height-ROLLER.height-PLATE_THICKNESS+2)* Rot(X=180)*BEARING)

STEPPER_CENTER1 = Location((20, 0, 0))
crane_plate1 = stepper_mount(crane_plate1, STEPPER, STEPPER_CENTER1, STEPPER_SHAFT_CIRCLE_RADIUS, STEPPER_FIXING_HOLES_DISTANCE)
print(find_circle_line_intersection(-28, 0, 46.039, 0))

STEPPER_CENTER2 = Location((find_circle_line_intersection(-28, 0, 46.039+2.7, 0)[1][0], 0, -2*SPACER6.height-ROLLER.height-PLATE_THICKNESS))
print(STEPPER_CENTER2)
crane_plate2 = Pos(Z = -2*SPACER6.height-ROLLER.height-PLATE_THICKNESS)*crane_plate2
crane_plate2 = stepper_mount(crane_plate2, STEPPER, STEPPER_CENTER2, STEPPER_SHAFT_CIRCLE_RADIUS, STEPPER_FIXING_HOLES_DISTANCE)


ROLLERS_CENTER = Location((-13, 0, crane_plate1.bounding_box().max.Z))
crane_plate1, rollers = plate_to_cariage(crane_plate1, PROFILE2020, 52, ROLLERS_CENTER, M6_round, ROLLER, SPACER6)
crane_plate2, _ = plate_to_cariage(crane_plate2, PROFILE2020, 52, ROLLERS_CENTER, M6_round, ROLLER, SPACER6)

export_stl(crane_plate1, "parts/crane_plate1.stl")
export_stl(crane_plate2, "parts/crane_plate2.stl")
#stepper = STEPPER_CENTER*Pos(Z=STEPPER.case_height+PLATE_THICKNESS/2)*Rot(X=180)*STEPPER.model
stepper1 = STEPPER_CENTER1*Pos(Z=STEPPER.case_height)*Rot(X=180)*STEPPER.model
stepper2 = STEPPER_CENTER2*Pos(Z=STEPPER.case_height)*Rot(X=180)*STEPPER.model

pulley_c = Pos(-28, 0,-73)*pulley
PULLEY = pulley_c
CRANE_CARRIAGE = Pos(Z=-PLATE_THICKNESS/2)*Compound(label="carriage", children = [crane_plate1, crane_plate2, *rollers, stepper1, stepper2])
export_stl(CRANE_CARRIAGE, "parts/crane_carriage_v2.stl")
show(CRANE_CARRIAGE, bearings)
# %%
#### ROBOT CARRIAGE
ROBOT_PLATE_HEIGHT = 110
ROBOT_PLATE_WIDTH = 70
STEPPER_CENTER = Location((30, 0, 0))
robot_plate = Box(ROBOT_PLATE_HEIGHT,ROBOT_PLATE_WIDTH , PLATE_THICKNESS)
robot_plate = fillet(robot_plate.edges().filter_by(Axis.Z), radius=7)


robot_plate1 = stepper_mount(robot_plate, STEPPER, STEPPER_CENTER, STEPPER_SHAFT_CIRCLE_RADIUS, STEPPER_FIXING_HOLES_DISTANCE)
ROLLERS_CENTER = Location((-15, 0, robot_plate.bounding_box().max.Z))

robot_plate1, _= plate_to_cariage(robot_plate1, PROFILE2040, 52, ROLLERS_CENTER, M6_round, ROLLER, SPACER6)

robot_plate2 = robot_plate
robot_plate2_spacer = Box(ROBOT_PLATE_HEIGHT,ROBOT_PLATE_WIDTH , 4)
robot_plate2_spacer = fillet(robot_plate2_spacer.edges().filter_by(Axis.Z), radius=7)
robot_plate2_spacer, _ = plate_to_cariage(robot_plate2_spacer, PROFILE2040, 52, ROLLERS_CENTER, M11, ROLLER, SPACER6)


robot_plate2, rollers =  plate_to_cariage(robot_plate2, PROFILE2040, 52, ROLLERS_CENTER, M6_round, ROLLER, SPACER6)
robot_plate2 = Pos(Z = -2*SPACER6.height-ROLLER.height-PLATE_THICKNESS)*Rot(X=180)*robot_plate2
rollers = [Pos(Z = -2*SPACER6.height-ROLLER.height-PLATE_THICKNESS)*Rot(X=180)*r for r in rollers]
robot_plate2_spacer = Pos(Z = -2*SPACER6.height-ROLLER.height-PLATE_THICKNESS-4)*Rot(X=180)*robot_plate2_spacer
robot_plate2 = robot_plate2 + robot_plate2_spacer
for i in [-22-15, 22-15]:
    for j in [-22, 22]:
        robot_plate2 = robot_plate2-(Pos(i, j)*ClearanceHole(fastener=cap_screw_m3, depth=500))
        robot_plate2 = robot_plate2-(Pos(i, j, -2*SPACER6.height-ROLLER.height-PLATE_THICKNESS)*Cylinder(5, PLATE_THICKNESS))

robot_plate1= robot_plate1 + Pos(-40, ROBOT_PLATE_WIDTH/2, PLATE_THICKNESS/2-0.5)*Rot(Y=90)*Box(1, 10, 20)
robot_plate1 = robot_plate1 + Pos(-40, -ROBOT_PLATE_WIDTH/2, PLATE_THICKNESS/2-0.5)*Rot(Y=90)*Box(1, 10, 20)
stepper = STEPPER_CENTER*Pos(Z=STEPPER.case_height+PLATE_THICKNESS/2)*Rot(X=180)*STEPPER.model
ROBOT_CARRIAGE = Pos(Z=PLATE_THICKNESS/2)*Rot(Y=180)*Compound(label="robot_carriage", children = [robot_plate1, robot_plate2, *rollers, stepper])
#robot_plate2.export_stl("parts/robot_plate2.stl")
#robot_plate1.export_stl("parts/robot_plate1.stl")

show(ROBOT_CARRIAGE)
# %%
a = 2
b = Box(20+2, 20+4, 40+2, align=[Align.MIN, Align.CENTER, Align.MIN])
b = b -Box(20, 20, 40, align=[Align.MIN, Align.CENTER, Align.MIN])
b = b-Pos(0, 0, 20)*Box(20, 12, 40, align=[Align.MIN, Align.CENTER, Align.MIN])
export_stl(b, "parts/cache.stl")
show(b)
# %%
r = Box(80, 20, PLATE_THICKNESS, align = [Align.CENTER, Align.CENTER, Align.MAX])
r = fillet(r.edges().filter_by(Axis.Z), radius=5)
s1 = Pos(X=30)*ClearanceHole(fastener=cap_screw_m4, depth=10, fit="Loose")
s2 = Pos(X=10)*ClearanceHole(fastener=cap_screw_m4, depth=10, fit="Loose")
s3 = Pos(X=-30)*ClearanceHole(fastener=cap_screw_m4, depth=10, fit="Loose")
s4 = Pos(X=-10)*ClearanceHole(fastener=cap_screw_m4, depth=10, fit="Loose")
r = r - s1 - s2 - s3 - s4
export_stl(r, "parts/plate_joint.stl")
show(r)

# %%
# %%
r = Box(60, 40, PLATE_THICKNESS, align = [Align.CENTER, Align.CENTER, Align.MAX])
r = fillet(r.edges().filter_by(Axis.Z), radius=5)
s1 = Pos(0, 10)*ClearanceHole(fastener=cap_screw_m4, depth=10)
s2 = Pos(-20, -10)*ClearanceHole(fastener=cap_screw_m4, depth=10)
s3 = Pos(20, -10)*ClearanceHole(fastener=cap_screw_m6, depth=10)
s4 = Pos(20, 10)*ClearanceHole(fastener=cap_screw_m6, depth=10)
r = r - s1 - s2 - s3 - s4
export_stl(r, "parts/plate_joint.stl")
show(r)

# %%
x, y, z = 16, 20, 30
endstop_holder_light_width = 3.46
support_height = 10
b = Box(x, y,z , align=(Align.CENTER, Align.CENTER, Align.MIN))
d = Pos(Z=5)*Box(endstop_holder_light_width, 20, 41, align=(Align.CENTER, Align.CENTER, Align.MIN))
t1 = Pos(0,-11,5)*extrude(Triangle(a=25, b=40, c=40), 28)
b = b - d -t1

l = Pos(0,15, 0)*Box(r_dim, support_height, 5,align=[Align.CENTER, Align.CENTER, Align.MIN])

l2 = Pos(10, 15,20)*ClearanceHole(fastener=cap_screw_m4, depth=100)
l3 = Pos(-10, 15,20)*ClearanceHole(fastener=cap_screw_m4, depth=100)
l4 = Pos(10, 15,3)*ClearanceHole(fastener=cap_screw_m8, depth=100)
l5 = Pos(-10, 15,3)*ClearanceHole(fastener=cap_screw_m8, depth=100)
l = l -l2 - l3 - l4 - l5

r1= Pos(-8+ (x -endstop_holder_light_width)/4, 10, 10)*Box((x -endstop_holder_light_width)/2, 2.15, 20, align=[Align.CENTER, Align.MIN, Align.MIN])
r2= Pos(8- (x -endstop_holder_light_width)/4, 10, 10)*Box((x -endstop_holder_light_width)/2, 2.15, 20, align=[Align.CENTER, Align.MIN, Align.MIN])
r3 = Pos(0, 10, 30-2.5)*Box(16, 2.15, 2.5, align=[Align.CENTER, Align.MIN, Align.MIN])
#r4= Pos(-8+ (x -endstop_holder_light_width)/4, 10, 0)*Box((x -endstop_holder_light_width)/2, 0.5, 20, align=[Align.CENTER, Align.MIN, Align.MIN])
#r5= Pos(8- (x -endstop_holder_light_width)/4, 10, 0)*Box((x -endstop_holder_light_width)/2+2, 0.5, 20, align=[Align.CENTER, Align.MIN, Align.MIN])
show(b, l, r1, r2, r3)
export_stl(b+l+r1+r2+r3, "parts/endstop_sheath.stl")


# %%

# %%
