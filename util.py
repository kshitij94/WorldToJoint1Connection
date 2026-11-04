import math
import os
from typing import Optional
import adsk.core
import adsk.fusion

BUFFER = 0.0  # mm
SEPARATOR = 1.0  # mm


def create_component(design: adsk.fusion.Design, name: str) -> adsk.fusion.Component:
    rootComp = design.rootComponent
    occ = rootComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = name
    return comp


def create_sketch_on_offset_plane(comp: adsk.fusion.Component, offset_mm: float = 0.0) -> adsk.fusion.Sketch:
    """
    Creates a construction plane offset from the XY plane, and returns a new sketch on it.

    :param comp: The component to create the plane and sketch in.
    :param offset_mm: The offset distance in mm.
    :return: The created Sketch object.
    """
    sketches = comp.sketches
    constructionPlanes = comp.constructionPlanes
    planeInput = constructionPlanes.createInput()
    # Fusion API uses cm internally, so convert mm to cm
    offsetValue = adsk.core.ValueInput.createByReal(offset_mm / 10.0)
    planeInput.setByOffset(comp.xYConstructionPlane, offsetValue)
    offsetPlane = constructionPlanes.add(planeInput)
    return sketches.add(offsetPlane)


def find_profile_by_outer_curve(
    sketch: adsk.fusion.Sketch, outer_curve: adsk.fusion.SketchEntity
) -> Optional[adsk.fusion.Profile]:
    """
    Finds the profile in a sketch that has the specified outer_curve as its outer loop.
    """
    outer_token = getattr(outer_curve, "entityToken", None)
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        for j in range(prof.profileLoops.count):
            loop = prof.profileLoops.item(j)
            if loop.isOuter:
                for k in range(loop.profileCurves.count):
                    pc = loop.profileCurves.item(k)
                    se = pc.sketchEntity
                    if se == outer_curve or (outer_token and getattr(se, "entityToken", None) == outer_token):
                        return prof
    return None


def extrude_profile(
    profile,
    distance_mm: float,
    operation: adsk.fusion.FeatureOperations = adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
    is_symmetric: bool = False,
) -> adsk.fusion.ExtrudeFeature:
    """
    Extrudes a profile, profile collection, or all profiles in a sketch by the specified distance in millimeters.
    """
    if isinstance(profile, adsk.fusion.Sketch):
        sketch = profile
        component = sketch.parentComponent
        profiles_coll = adsk.core.ObjectCollection.create()
        for i in range(sketch.profiles.count):
            profiles_coll.add(sketch.profiles.item(i))
        profile_input = profiles_coll
    elif isinstance(profile, adsk.core.ObjectCollection):
        component = profile.item(0).parentSketch.parentComponent
        profile_input = profile
    else:
        component = profile.parentSketch.parentComponent
        profile_input = profile

    extrudes = component.features.extrudeFeatures
    distance = adsk.core.ValueInput.createByReal(distance_mm / 10.0)
    extrude_input = extrudes.createInput(profile_input, operation)
    extrude_input.setDistanceExtent(is_symmetric, distance)
    return extrudes.add(extrude_input)


def create_circle(
    sketch: adsk.fusion.Sketch, center: adsk.core.Point3D, radius_mm: float
) -> adsk.fusion.SketchCircle:
    circles = sketch.sketchCurves.sketchCircles
    circle = circles.addByCenterRadius(center, radius_mm / 10.0)
    circle.isFixed = True
    return circle


def draw_circles_pattern(
    sketch: adsk.fusion.Sketch,
    center_point: Optional[adsk.core.Point3D] = None,
    pcd_mm: float = 170.0,
    hole_diameter_mm: float = 6.0,
    hole_count: int = 3,
    start_angle_deg: float = 60.0,
    draw_pcd_construction: bool = False,
) -> list:
    """Draws a circular pattern of holes along a PCD circle on a sketch with an optional angular offset."""
    if center_point is None:
        center_point = adsk.core.Point3D.create(0, 0, 0)

    circles = sketch.sketchCurves.sketchCircles
    drawn_circles = []

    # PCD radius in cm
    screw_pcd_radius_cm = (pcd_mm / 10.0) / 2.0

    # Optionally draw PCD construction circle
    if draw_pcd_construction:
        screw_circle = circles.addByCenterRadius(center_point, screw_pcd_radius_cm)
        screw_circle.isConstruction = True

    # Holes along the PCD circle
    hole_radius_cm = (hole_diameter_mm / 10.0) / 2.0
    start_angle_rad = math.radians(start_angle_deg)
    for i in range(hole_count):
        angle = start_angle_rad + (2.0 * math.pi * i / hole_count)
        x = center_point.x + screw_pcd_radius_cm * math.cos(angle)
        y = center_point.y + screw_pcd_radius_cm * math.sin(angle)
        hole_center = adsk.core.Point3D.create(x, y, center_point.z)
        c = circles.addByCenterRadius(hole_center, hole_radius_cm)
        c.isFixed = True
        drawn_circles.append(c)

    return drawn_circles


def find_profiles_in_circle(
    sketch: adsk.fusion.Sketch, center: adsk.core.Point3D, radius: float
) -> adsk.core.ObjectCollection:
    """
    Finds all profiles in a sketch whose centroids lie within the specified circle.
    """
    profiles = adsk.core.ObjectCollection.create()
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        try:
            area_props = prof.areaProperties()
            centroid = area_props.centroid
            dist = math.sqrt((centroid.x - center.x) ** 2 + (centroid.y - center.y) ** 2)
            if dist < radius + 1e-5:
                profiles.add(prof)
        except:
            pass
    return profiles


def get_body_height(comp: adsk.fusion.Component) -> float:
    """
    Returns the maximum height (Z-axis extent) of the solid bodies in the component in mm.
    """
    if comp.bRepBodies.count == 0:
        return 0.0

    min_z = float("inf")
    max_z = float("-inf")

    for i in range(comp.bRepBodies.count):
        body = comp.bRepBodies.item(i)
        bbox = body.boundingBox
        if bbox.minPoint.z < min_z:
            min_z = bbox.minPoint.z
        if bbox.maxPoint.z > max_z:
            max_z = bbox.maxPoint.z

    if min_z == float("inf") or max_z == float("-inf"):
        return 0.0

    return (max_z - min_z) * 10.0


def extrudeMaxAreaProfile(
    sketch: adsk.fusion.Sketch,
    extrudeDistanceMM: float,
    comp: Optional[adsk.fusion.Component] = None,
    operation: adsk.fusion.FeatureOperations = adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
) -> Optional[adsk.fusion.ExtrudeFeature]:
    """
    Finds the profile with the largest area in the sketch and extrudes it by the specified distance in mm.
    """
    if isinstance(sketch, adsk.fusion.Component) and isinstance(extrudeDistanceMM, adsk.fusion.Sketch):
        comp, sketch, extrudeDistanceMM = sketch, extrudeDistanceMM, comp  # type: ignore

    if comp is None:
        comp = sketch.parentComponent

    base_prof = None
    max_area = 0.0
    for i in range(sketch.profiles.count):
        prof = sketch.profiles.item(i)
        try:
            area = prof.areaProperties().area
            if area > max_area:
                max_area = area
                base_prof = prof
        except:
            pass

    if not base_prof and sketch.profiles.count > 0:
        base_prof = sketch.profiles.item(0)

    if base_prof:
        extrudes = comp.features.extrudeFeatures
        extInput = extrudes.createInput(base_prof, operation)
        distance = adsk.core.ValueInput.createByReal(extrudeDistanceMM / 10.0)
        extInput.setDistanceExtent(False, distance)
        return extrudes.add(extInput)
    return None


def save_stl(design: adsk.fusion.Design, components_to_export: list) -> list:
    """
    Exports a list of (Component, name) tuples as STL files to the Downloads directory.
    """
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

