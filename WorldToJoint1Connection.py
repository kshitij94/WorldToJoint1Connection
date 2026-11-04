"""World to Joint 1 Connection Script for Autodesk Fusion 360."""

import importlib
import math
import os
import sys
import traceback
import adsk.core
import adsk.fusion

# Ensure local modules can be imported
script_dir = os.path.dirname(__file__)
if script_dir in sys.path:
    sys.path.remove(script_dir)
sys.path.insert(0, script_dir)

import util
importlib.reload(util)

BASE_SKETCH_OFFSET = 0.0  # mm
INNER_CIRCLE_PCD_MM = 80.0  # mm
MIDDLE_CIRCLE_PCD_MM = 140.0  # mm
OUTER_CIRCLE_PCD_MM = 140.0 + 30.0 + 30.0  # mm
OUTER_CIRCLE_EXTRUDE_DISTANCE_MM = 5.0  # mm
MIDDLE_CIRCLE_EXTRUDE_DISTANCE_MM = 85.0  # mm

# Support settings: 3 triangular supports of 30 degrees each, equally spaced (120 deg apart)
SUPPORT_COUNT = 3
SUPPORT_ANGLE_DEG = 30.0  # degrees width of each support
SUPPORT_START_ANGLE_DEG = 0.0  # starting angle in degrees

# Mounting holes on outer flange (for wood screws to base below, placed in the unsupported gaps)
MOUNTING_HOLE_DIAMETER_MM = 6.0  # mm (hole size for screw)
MOUNTING_HOLE_PCD_MM = (MIDDLE_CIRCLE_PCD_MM + OUTER_CIRCLE_PCD_MM) / 2.0  # 170.0 mm (centered on flange)
MOUNTING_HOLE_COUNT = 3
MOUNTING_HOLE_START_ANGLE_DEG = 60.0  # degrees (centered between supports at 0 and 120 deg)

app = adsk.core.Application.get()
ui = app.userInterface


def construct_triangular_supports(
    comp: adsk.fusion.Component,
    count: int = SUPPORT_COUNT,
    support_angle_deg: float = SUPPORT_ANGLE_DEG,
    start_angle_deg: float = SUPPORT_START_ANGLE_DEG,
) -> list:
    """Constructs triangular revolute supports connecting the middle cylinder top

    to the outer cylinder edge at equal angular intervals.
    """
    revolve_features = []
    if count <= 0:
        return revolve_features

    r_mid_cm = (MIDDLE_CIRCLE_PCD_MM / 2.0) / 10.0
    r_outer_cm = (OUTER_CIRCLE_PCD_MM / 2.0) / 10.0
    z_top_cm = MIDDLE_CIRCLE_EXTRUDE_DISTANCE_MM / 10.0
    z_flange_cm = OUTER_CIRCLE_EXTRUDE_DISTANCE_MM / 10.0

    angle_step_deg = 360.0 / count
    half_width_rad = math.radians(support_angle_deg / 2.0)

    for i in range(count):
        angle_deg = (start_angle_deg + i * angle_step_deg) % 360.0

        # Create or select the plane for this angle around the Z axis
        if abs(angle_deg) < 1e-4:
            plane = comp.xZConstructionPlane
        else:
            plane_input = comp.constructionPlanes.createInput()
            plane_input.setByAngle(
                comp.zConstructionAxis,
                adsk.core.ValueInput.createByReal(math.radians(angle_deg)),
                comp.xZConstructionPlane,
            )
            plane = comp.constructionPlanes.add(plane_input)

        sketch = comp.sketches.add(plane)

        # Compute 3D points on the plane at angle_deg
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        # Top of middle cylinder outer wall
        pt_top = adsk.core.Point3D.create(r_mid_cm * cos_a, r_mid_cm * sin_a, z_top_cm)
        # Outer cylinder outer edge on top of base flange
        pt_outer = adsk.core.Point3D.create(r_outer_cm * cos_a, r_outer_cm * sin_a, z_flange_cm)
        # Corner between middle cylinder wall and base flange top
        pt_corner = adsk.core.Point3D.create(r_mid_cm * cos_a, r_mid_cm * sin_a, z_flange_cm)

        # Project 3D points to 2D sketch space
        sk_pt_top = sketch.modelToSketchSpace(pt_top)
        sk_pt_outer = sketch.modelToSketchSpace(pt_outer)
        sk_pt_corner = sketch.modelToSketchSpace(pt_corner)

        # Draw triangular profile
        lines = sketch.sketchCurves.sketchLines
        lines.addByTwoPoints(sk_pt_top, sk_pt_outer)
        lines.addByTwoPoints(sk_pt_outer, sk_pt_corner)
        lines.addByTwoPoints(sk_pt_corner, sk_pt_top)

        if sketch.profiles.count > 0:
            prof = sketch.profiles.item(0)
            revolves = comp.features.revolveFeatures
            rev_input = revolves.createInput(
                prof,
                comp.zConstructionAxis,
                adsk.fusion.FeatureOperations.JoinFeatureOperation,
            )
            # Revolve symmetrically by half_width_rad on each side (total width = support_angle_deg)
            rev_input.setAngleExtent(True, adsk.core.ValueInput.createByReal(half_width_rad))
            rev_feat = revolves.add(rev_input)
            revolve_features.append(rev_feat)

    return revolve_features


def construct_mounting_holes(
    comp: adsk.fusion.Component,
    pcd_mm: float = MOUNTING_HOLE_PCD_MM,
    hole_diameter_mm: float = MOUNTING_HOLE_DIAMETER_MM,
    count: int = MOUNTING_HOLE_COUNT,
    start_angle_deg: float = MOUNTING_HOLE_START_ANGLE_DEG,
):
    """Cuts through-holes in the outer flange for screwing to the base below,

    positioned in the unsupported angular gaps.
    """
    holes_sketch = comp.sketches.add(comp.xYConstructionPlane)
    util.draw_circles_pattern(
        sketch=holes_sketch,
        pcd_mm=pcd_mm,
        hole_diameter_mm=hole_diameter_mm,
        hole_count=count,
        start_angle_deg=start_angle_deg,
    )
    if holes_sketch.profiles.count > 0:
        util.extrude_profile(
            holes_sketch,
            OUTER_CIRCLE_EXTRUDE_DISTANCE_MM + 5.0,
            operation=adsk.fusion.FeatureOperations.CutFeatureOperation,
        )


def run(context):
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design found.", "No Design")
            return

        base_comp = util.create_component(design, "base")
        sketch = util.create_sketch_on_offset_plane(base_comp, BASE_SKETCH_OFFSET)

        origin = adsk.core.Point3D.create(0, 0, 0)
        inner_circle = util.create_circle(sketch, origin, INNER_CIRCLE_PCD_MM / 2.0)
        middle_circle = util.create_circle(sketch, origin, MIDDLE_CIRCLE_PCD_MM / 2.0)
        outer_circle = util.create_circle(sketch, origin, OUTER_CIRCLE_PCD_MM / 2.0)

        outer_profile = util.find_profile_by_outer_curve(sketch, outer_circle)
        if outer_profile:
            util.extrude_profile(outer_profile, OUTER_CIRCLE_EXTRUDE_DISTANCE_MM)

        middle_profile = util.find_profile_by_outer_curve(sketch, middle_circle)
        if middle_profile:
            util.extrude_profile(
                middle_profile,
                MIDDLE_CIRCLE_EXTRUDE_DISTANCE_MM,
                operation=adsk.fusion.FeatureOperations.JoinFeatureOperation,
            )

        # Full body cut of the inner circle (80mm PCD)
        inner_profile = util.find_profile_by_outer_curve(sketch, inner_circle)
        if inner_profile:
            try:
                util.extrude_profile(
                    inner_profile,
                    MIDDLE_CIRCLE_EXTRUDE_DISTANCE_MM,
                    operation=adsk.fusion.FeatureOperations.CutFeatureOperation,
                )
            except Exception:
                # The inner 80mm region is already naturally hollow from the annular profile extrusions
                pass

        # 3 triangular revolute supports (30 degrees each, spaced at 120 degrees)
        construct_triangular_supports(base_comp)

        # 3 mounting holes (6mm diameter) on the outer flange in the unsupported gaps (at 60, 180, 300 deg)
        construct_mounting_holes(base_comp)

    except:
        if ui:
            ui.messageBox(f"Failed:\n{traceback.format_exc()}")


def stop(context):
    pass

