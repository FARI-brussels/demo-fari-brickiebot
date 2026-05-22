# %%
from build123d import *
from ocp_vscode import *

def create_press_fit_on_edge(f, e, n, d, type="male"):
    for i in range(n):
        r = Pos(e.position_at((i + 1) / (n + 1))) * Rectangle(2*d, 2*d)
        if type == "male":
            f = f + r
        elif type == "female":
            f = f - r
    return f

# Function to create press-fit notches on a face
def create_press_fit_on_face(f, n, d, type="male"):
    if type == "male" or type == "female":
        for edge in f.edges():
            f = create_press_fit_on_edge(f, edge, n,d, type)
    elif type == "malefemale":
        for i, edge in enumerate(f.edges()):
            if i % 2 == 0:
                f = create_press_fit_on_edge(f, edge, n,d, "male" )
            else:
                f = create_press_fit_on_edge(f, edge, n,d, 'female')
    elif type =="femalemale":
        for i, edge in enumerate(f.edges()):
            if i % 2 == 0:
                f = create_press_fit_on_edge(f, edge, n,d, "female" )
            else:   
                f = create_press_fit_on_edge(f, edge, n,d, "male")
    return f
length = 80
width = 45
height = 45
thickness = 3
z = Rectangle(length, width)

#x1 = Pos(0, 0, height/2+thickness)*Rot(Y=90)*Rectangle(height+2*thickness, width)
x = Pos(Z=height/2+thickness)*Rot(Y=90)*create_press_fit_on_face(Rectangle(height+2*thickness, width), thickness, thickness, type="femalemale")
y = Pos(Z=height/2+thickness)*Rot(X=90)*Rectangle( length+2*thickness, height+2*thickness)
y = extrude(y, thickness)
z = create_press_fit_on_face(z, thickness, thickness)
z = extrude(z, thickness)
z1 = z
z2 = Pos(Z=height+thickness)*z
x = extrude(x, thickness)
x1 = Pos(-length/2-thickness, 0)*x
x2 = Pos(length/2, 0)*x

y1 = Pos(Y=-width/2) * y
y2 = Pos(Y=width/2+thickness) * y
c = Pos(Z=height+thickness)*Box(*3*(2*max(height, width, length),), align=[Align.CENTER, Align.CENTER, Align.MIN])

y1 = y1 - x1 - x2 -z1 - z2 - c
y2 = y2 - x1 - x2 -z1 - z2 - c
x1 = x1-c
x2 = x2-c
# Create the press-fit box with extruded faces
print(3*(max(height, width, length),))
# Display the box
show(z1,  x1, x2, y1, y2)
export_stl(z1, "z.stl")
export_stl(x1, "x.stl")
export_stl(y1, "y.stl")
# %%
x = Pos(Z=height/2+thickness)*Rot(Y=90)*Rectangle(height+2*thickness, width)
x = extrude(x, thickness)
y = Pos(Z=height/2+thickness)*Rot(X=90)*Rectangle( length+2*thickness, height+2*thickness)
y = extrude(y, thickness)
z = Rectangle(length, width)
z = extrude(z, thickness)

z1 = z
z2 = Pos(Z=height+thickness)*z

x1 = Pos(-length/2-thickness, 0)*x
x2 = Pos(length/2, 0)*x

y1 = Pos(Y=-width/2) * y
y2 = Pos(Y=width/2+thickness) * y


fari = import_svg("farilogo.svg")
fari = scale(import_svg("farilogo.svg"), 0.11)
fari = Pos(-31, -268, 0)*fari
fari = Rot(X=180)*fari
fari = extrude(fari, amount=-height/5)

z1_fari = z1+fari
a = Compound([z1, x1, y1])
a_fari = Compound([z1_fari, x1, y1])
b = Compound([x2, y2])

show(a_fari,b, fari)
export_stl(a_fari, "a_fari.stl")
export_stl(a, "a.stl")
export_stl(b, "b.stl")
export_stl(fari, "fari.stl")


# %%
c = Compound([x1, y1])
d = Compound([x2, y2])
e_fari = z1_fari
show(c, d, e_fari, fari)

export_stl(c, "c.stl")
export_stl(d, "d.stl")
export_stl(e_fari, "e_fari.stl")
 # %%
