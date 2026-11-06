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
INNER_CIRCLE_PCD_MM = 80.5  # mm
MIDDLE_CIRCLE_PCD_MM = 144.0  # mm
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

# Wire pass-through hole (cuts through the middle cylinder wall at the bottom)
WIRE_HOLE_DIAMETER_MM = 12.0  # mm (diameter of wire exit hole)
WIRE_HOLE_CENTER_Z_MM = OUTER_CIRCLE_EXTRUDE_DISTANCE_MM + (WIRE_HOLE_DIAMETER_MM / 2.0)  # 11.0 mm (bottom touches flange top)
WIRE_HOLE_ANGLE_DEG = 180.0  # degrees (back of cylinder, centered between 120 and 240 deg supports)

# 6 Cylinder screw holes (6mm diameter on PCD 134mm)
CYLINDER_SCREW_HOLE_PCD_MM = 134.0  # mm
CYLINDER_SCREW_HOLE_DIAMETER_MM = 6.0  # mm
CYLINDER_SCREW_HOLE_COUNT = 6
CYLINDER_SCREW_HOLE_START_ANGLE_DEG = 30.0  # degrees (interleaved between supports at 0, 120, 240 deg)

# STL export settings
STL_EXPORT_NAME = "WorldToJoint1_Base"

app = adsk.core.Application.get()
ui = app.userInterface


def save_stl(design: adsk.fusion.Design, components_to_export: list) -> list:
    """Exports components as STL files into the user's Downloads directory (matching joint2_joint3_connector)."""
    downloads_dir = os.path.join(os.path.expanduser("~"), "Downloads")
    if not os.path.exists(downloads_dir):
        downloads_dir = os.path.join(os.path.expanduser("~"), "Desktop")

    export_mgr = design.exportManager
    exported_paths = []
    for comp, name in components_to_export:
        file_path = os.path.join(downloads_dir, f"{name}.stl")
        stl_options = export_mgr.createSTLExportOptions(comp, file_path)
        stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        stl_options.sendToPrintUtility = False
        export_mgr.execute(stl_options)
        exported_paths.append(file_path)
    return exported_paths


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


def construct_wire_hole(
    comp: adsk.fusion.Component,
    hole_diameter_mm: float = WIRE_HOLE_DIAMETER_MM,
    center_z_mm: float = WIRE_HOLE_CENTER_Z_MM,
    angle_deg: float = WIRE_HOLE_ANGLE_DEG,
):
    """Cuts a horizontal wire exit hole through the middle cylinder wall at the bottom."""
    extrudes = comp.features.extrudeFeatures
    r_cut_cm = (MIDDLE_CIRCLE_PCD_MM / 2.0 + 15.0) / 10.0
    hole_radius_cm = (hole_diameter_mm / 2.0) / 10.0
    center_z_cm = center_z_mm / 10.0

    # Determine sketch plane and cut distance along the radial direction of angle_deg
    if abs((angle_deg - 180.0) % 360.0) < 1e-4:
        sketch_plane = comp.yZConstructionPlane
        cut_distance_cm = -r_cut_cm
    elif abs(angle_deg % 360.0) < 1e-4:
        sketch_plane = comp.yZConstructionPlane
        cut_distance_cm = r_cut_cm
    elif abs((angle_deg - 90.0) % 360.0) < 1e-4:
        sketch_plane = comp.xZConstructionPlane
        cut_distance_cm = r_cut_cm
    elif abs((angle_deg - 270.0) % 360.0) < 1e-4:
        sketch_plane = comp.xZConstructionPlane
        cut_distance_cm = -r_cut_cm
    else:
        # Create a construction plane through the Z-axis whose normal points along angle_deg
        plane_input = comp.constructionPlanes.createInput()
        plane_input.setByAngle(
            comp.zConstructionAxis,
            adsk.core.ValueInput.createByReal(math.radians(angle_deg - 90.0)),
            comp.xZConstructionPlane,
        )
        sketch_plane = comp.constructionPlanes.add(plane_input)
        cut_distance_cm = r_cut_cm

    sketch = comp.sketches.add(sketch_plane)
    center_pt = sketch.modelToSketchSpace(adsk.core.Point3D.create(0.0, 0.0, center_z_cm))
    circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(center_pt, hole_radius_cm)
    circle.isFixed = True

    prof = util.find_profile_by_outer_curve(sketch, circle)
    if not prof and sketch.profiles.count > 0:
        prof = sketch.profiles.item(0)

    if prof:
        cut_input = extrudes.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
        cut_input.setDistanceExtent(False, adsk.core.ValueInput.createByReal(cut_distance_cm))
        extrudes.add(cut_input)


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

        # 6 smaller circles (6mm diameter on PCD 134mm) for screw holes
        util.draw_circles_pattern(
            sketch=sketch,
            center_point=origin,
            pcd_mm=CYLINDER_SCREW_HOLE_PCD_MM,
            hole_diameter_mm=CYLINDER_SCREW_HOLE_DIAMETER_MM,
            hole_count=CYLINDER_SCREW_HOLE_COUNT,
            start_angle_deg=CYLINDER_SCREW_HOLE_START_ANGLE_DEG,
        )

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

        # Full body cut of the inner circle (80.5mm PCD)
        inner_profile = util.find_profile_by_outer_curve(sketch, inner_circle)
        if inner_profile:
            try:
                util.extrude_profile(
                    inner_profile,
                    MIDDLE_CIRCLE_EXTRUDE_DISTANCE_MM,
                    operation=adsk.fusion.FeatureOperations.CutFeatureOperation,
                )
            except Exception:
                # The inner 80.5mm region is already naturally hollow from the annular profile extrusions
                pass

        # 3 triangular revolute supports (30 degrees each, spaced at 120 degrees)
        construct_triangular_supports(base_comp)

        # 3 mounting holes (6mm diameter) on the outer flange in the unsupported gaps (at 60, 180, 300 deg)
        construct_mounting_holes(base_comp)

        # Wire pass-through hole (12mm diameter) through the middle cylinder wall at the bottom
        construct_wire_hole(base_comp)

        # Export component as STL for 3D printing
        components_to_export = [
            (base_comp, STL_EXPORT_NAME),
        ]
        save_stl(design, components_to_export)

    except:
        if ui:
            ui.messageBox(f"Failed:\n{traceback.format_exc()}")


def stop(context):
    pass

