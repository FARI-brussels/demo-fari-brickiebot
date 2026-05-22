from build123d import *


def cad_path(path):
    return str(path)


class StepperMotor:
    def __init__(self, case_height=None, width=None, total_height=None, step_file=None, color=None, source=None):
        if step_file:
            self.model = import_step(cad_path(step_file))
            self.total_height = self.model.bounding_box().size.Z
            self.case_height = case_height
            self.width = self.model.bounding_box().size.X
        else : 
            self.total_height = total_height
            self.case_height = case_height
            self.width = width

        self.source = source
        if color : 
            self.model.color = color
    def source(self):
        pass

class AluminiumExtrustion:
    def __init__(self, step_file, color=Color("gray"), source=None):
        self.profile = import_step(cad_path(step_file))
        self.width = self.profile.bounding_box().size.X
        self.height = self.profile.bounding_box().size.Y
        self.source = source
        self.color = color
        

    def source(self):
        pass

    def extrude(self, height):
        p = extrude(self.profile.faces().sort_by().first, height)
        p = Pos(-p.center())*p
        p.color = self.color
        return p
    
class Roller:
    def __init__(self, step_file, rolling_diameter, rolling_center_offset, color="black", source=None):
        self.roller = import_step(cad_path(step_file))
        self.roller.color = color
        self.color = color
        self.rolling_diameter = rolling_diameter
        self.rolling_center_offset = rolling_center_offset
        self.diameter = self.roller.bounding_box().size.X
        self.height = self.roller.bounding_box().size.Z

    
    @property
    def model(self):
        return self.roller
    
class Spacer:
    def __init__(self, height, outer_diameter,inner_diameter, color="grey", source=None):
        self.spacer = Cylinder(outer_diameter/2, height, align=[Align.CENTER, Align.CENTER, Align.MAX]) - Cylinder(inner_diameter/2, height, align=[Align.CENTER, Align.CENTER, Align.MAX])
        self.spacer.color= color
        self.height = height

    @property
    def model(self):
        return self.spacer
    


def stepper_mount(plate, stepper, stepper_center, stepper_shaft_circle_radius, stepper_hole_distance):
    plate_thickness = plate.bounding_box().size.Z
    h_stepper_screws = Cylinder(1.55, plate_thickness)
    h_stepper_shaft = stepper_center*Cylinder(stepper_shaft_circle_radius, plate_thickness)
    plate = plate - h_stepper_shaft
    for i in [-stepper_hole_distance, stepper_hole_distance]:
        for j in [-stepper_hole_distance, stepper_hole_distance]:
            h= Pos(i,j)*stepper_center*h_stepper_screws
            plate = plate - h
    return plate


def plate_to_cariage(plate, profile, rollers_spacing, mounting_holes_origin, screw, roller, spacer, label=None):
    plate_thickness = plate.bounding_box().size.Z
    l = profile.height/2 + roller.rolling_diameter/2
    rollers = []
    for i in [-l, l]:
        for j in [-rollers_spacing/2, rollers_spacing/2]:
            sc = Pos(i,j)*mounting_holes_origin*screw
            sp = Pos(i,j, - plate_thickness)*mounting_holes_origin*spacer.model
            r = Pos(i,j, -roller.height/2- plate_thickness - spacer.height)*mounting_holes_origin*roller.model
            sp2 = Pos(i,j, - plate_thickness-roller.height-spacer.height)*mounting_holes_origin*spacer.model
            plate = plate- sc
            rollers.append(sc)
            rollers.append(sp)
            rollers.append(r)
            rollers.append(sp2)

    return plate, rollers


def calculate_pulley_center_distance(d1, d2, belt_length):
    """
    Calculate the center distance between two pulleys.
    
    Args:
        d1 (float): Diameter of first pulley
        d2 (float): Diameter of second pulley
        belt_length (float): Total length of the belt
    
    Returns:
        float: Center distance between pulleys, None if no valid solution exists
    """
    # Check for valid inputs
    if d1 <= 0 or d2 <= 0 or belt_length <= 0:
        return None
        
    # Convert to radii
    r1 = d1 / 2
    r2 = d2 / 2
    
    # Solve quadratic equation: C² - LC/2 + (r1 - r2)² = 0
    a = 1
    b = -belt_length/2
    c = (r1 - r2)**2
    
    # Calculate discriminant
    discriminant = b**2 - 4*a*c
    
    if discriminant >= 0:
        c1 = (-b + (discriminant)**0.5) / (2*a)
        c2 = (-b - (discriminant)**0.5) / (2*a)
        
        # Find valid solution (positive and larger than |r1 - r2|)
        min_distance = abs(r1 - r2)
        solutions = [c for c in [c1, c2] if c > min_distance]
        
        return min(solutions) if solutions else None
    return None


def find_circle_line_intersection(center_x, center_y, radius, line_y):
    # Calculate vertical distance from center to line
    distance = abs(line_y - center_y)
    
    # If distance is greater than radius, no intersection
    if distance > radius:
        return []
    
    # If distance equals radius (within floating point precision), tangent point
    if abs(distance - radius) < 1e-10:
        return [(center_x, line_y)]
        
    # Calculate the horizontal distance from center to intersection points
    h = (radius**2 - distance**2)**0.5
    x1 = center_x + h
    x2 = center_x - h
    
    # Return intersection points
    return [(x1, line_y), (x2, line_y)]
