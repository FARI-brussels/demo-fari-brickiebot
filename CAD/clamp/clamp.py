# %%
from build123d import *
from ocp_vscode import *
from bd_warehouse.bearing import SingleRowDeepGrooveBallBearing, PressFitHole
from bd_warehouse.fastener import SetScrew, CounterSunkScrew, ClearanceHole, HexNut
AZ_MIN = (Align.CENTER, Align.CENTER, Align.MIN)
PLATE_THICKNESS = 4

BEARING = SingleRowDeepGrooveBallBearing(size=f"M3-10-4")
NUT = HexNut(size="M3-0.5")
SCREW = SetScrew(size="M3-0.5", length=10)
latte = Box(100, 20, PLATE_THICKNESS, align = AZ_MIN)
latte = fillet(latte.edges().filter_by(Axis.Z), radius=9.99)
latte = fillet(latte.edges(), 1)
bearing_Z = PLATE_THICKNESS- 4
bearing1 = Pos(-40, 0, bearing_Z)*BEARING
bearing2 = Pos(40, 0, bearing_Z)*BEARING


latte1 = latte - Pos(40, 0, PLATE_THICKNESS)*PressFitHole(bearing=BEARING, depth=0.00001)- Pos(-40, 0, PLATE_THICKNESS)*PressFitHole(bearing=BEARING, depth=0.00001)
latte2 = latte - Pos(40, 0, PLATE_THICKNESS)*PressFitHole(bearing=BEARING, depth=0.00001) - Pos(-40, 0, PLATE_THICKNESS)*(ClearanceHole(fastener=NUT, depth=0, captive_nut=True) + ClearanceHole(fastener=SCREW, depth=20))
export_stl(latte1, 'latte1.stl')
export_stl(latte2, 'latte2.stl')
latte1 = Compound(children=[latte1, bearing1, bearing2], label="double bearing")
latte2 = Compound(children=[latte2, bearing2])
show(latte1, Pos(Y=20)*latte2)

# %%
pts = [
    (0, 0),
    (70, -30),
    (50, -60), 
]

segment_count = 6
s = Spline(pts, 
           tangents=((1, -0.00000001, 0), (-1, -1, 0)),)
handle_path = s.edges()[0] 
sections = Sketch()
for i in range(segment_count + 1):
    plane = Plane(
        origin=handle_path @ (i / segment_count),
        z_dir=handle_path % (i / segment_count),
    )
    rectangle = plane * Rectangle(20, PLATE_THICKNESS, align = AZ_MIN)
    sections += rectangle

# Create the handle by sweeping along the path
bent_stripe = Pos(Z=PLATE_THICKNESS/2)*sweep(sections, path=handle_path, multisection=True, is_frenet=True)
straight_half = Pos(X=-50)*(Box(100, 20, PLATE_THICKNESS,  align = AZ_MIN) -Pos(-40, 0, PLATE_THICKNESS) *PressFitHole(bearing=BEARING, depth=0.00001) -Pos(40, 0, PLATE_THICKNESS)*PressFitHole(bearing=BEARING, depth=0.00001))
bent_stripe_1 = Pos(X=10)*(bent_stripe + straight_half)
bent_stripe_1 = fillet(bent_stripe_1.edges().filter_by(Axis.Z), radius=9.99)
bent_stripe_1 = fillet(bent_stripe_1.edges()[16], radius = 9.99)
bent_stripe_1 = fillet(bent_stripe_1.edges()[19], radius = 9.99)

bent_stripe_1 = fillet(bent_stripe_1.edges(), 1)
show(bent_stripe_1)
export_stl(bent_stripe_1, "bent_stripe.stl")
# %%
## JAWS

b = Box(60, 30, PLATE_THICKNESS, align=AZ_MIN)
b = fillet(b.edges().filter_by(Axis.Z), radius=5)

h1 = Pos(-PLATE_THICKNESS, 0, PLATE_THICKNESS)*Box(PLATE_THICKNESS, 20, 20+1, align=AZ_MIN)
h1 = fillet(h1.edges().group_by(Axis.Z)[-1][2:4], radius=9.99)

h2 = Box(PLATE_THICKNESS, 20, 20+1, align=AZ_MIN)
h2 = Pos(Z=PLATE_THICKNESS)*fillet(h2.edges().group_by(Axis.Z)[-1][2:4], radius=9.99)

jaw1 = b + h1 + Pos(PLATE_THICKNESS, 0, 0)*h2
jaw1 = fillet(jaw1.edges(), 1)
jaw2 = b + h2 
jaw2 = fillet(jaw2.edges(), 1)
cap_screw =  SetScrew(size="M3-0.5", length=20)

jaw1 = jaw1 - Pos(10, 0, PLATE_THICKNESS+ 10+1)* Rot(Y=90)*ClearanceHole(fastener=cap_screw, depth=20)
jaw2 = jaw2 - Pos(10, 0, PLATE_THICKNESS+ 10+1)* Rot(Y=90)*ClearanceHole(fastener=cap_screw, depth=20)

latte = Pos(-PLATE_THICKNESS/2, 0, 100/2+ PLATE_THICKNESS)*Rot(Y=90)*latte
show(jaw1, latte, jaw2)

export_stl(jaw1, "jaw1.stl")
export_stl(jaw2, "jaw2.stl")

# %%
# ASSEMBLY
bent_stripe_2 = Rot(Y=180)*bent_stripe_1
RevoluteJoint("hinge_axis", bent_stripe_1, axis=Axis((0, 0, 0), (0, 0, 1)))
RevoluteJoint("hinge_axis", bent_stripe_2, axis=Axis((0, 0, 0), (0, 0, 1)))


show(bent_stripe_2, bent_stripe_1, jaw, render_joints=True)
# %%
RevoluteJoint("hinge_axis", bent_stripe_1, axis=Axis((0, 0, 0), (0, 0, 1)))
# %%

jaw2 = Pos(50, -60, PLATE_THICKNESS/2)*Rot(Z=20)*Rot(Y=90)*jaw
stripe_jaw= Compound(children=[bent_stripe_1, jaw2], label="stripe_jaw")
export_stl(stripe_jaw, "stripe_jaw.stl")
show(bent_stripe_1, jaw2)
# %%
# FIxtures 
l = Box(22, 10, 1.5)
l = fillet(l.edges().filter_by(Axis.Z), radius=4.99)
l = l - Pos(-11+3,0, 10)*ClearanceHole(fastener=cap_screw, depth=20)
l = l - Pos(11-5,0, 0)*Box(3, 6, 3)
export_stl(l, "clamp_fixture.stl")
show(l)

# %%
# FIxtures 90° orientation
height = 22
width = 6
thikness = 1.5
l = Box(height, width, thikness, align=AZ_MIN)
l = fillet(l.edges().filter_by(Axis.Z)[0:2], radius=2)

l = l - Pos(-11+3,0, 10)*ClearanceHole(fastener=cap_screw, depth=20)
l = Rot(Y=-90)*l
l2 = Pos(11.5, 0, 0)*l
l3 = Pos(5, 0, 8)*(Box(10, width, 6) - Box(6.1, width, 3))
clamp_fixture = Compound(children=[l, l2, l3])
export_stl(clamp_fixture, "clamp_fixture.stl")
show(clamp_fixture)

# %%


# %%
