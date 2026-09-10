"""Run inside UnrealEditor-Cmd with PythonScriptPlugin and EditorScriptingUtilities.

Creates only /Game/Tests/PBDChaosComparison, then exercises both actual solvers
without starting PIE. Existing levels/default maps are not modified.
"""
import json
from pathlib import Path
import re

import unreal


MAP_PATH = "/Game/Tests/PBDChaosComparison"
comparison_class = unreal.load_class(None, "/Script/pdd_cloth.PBDClothComparisonActor")
if comparison_class is None:
    raise RuntimeError("Build the new project module before running this script.")

levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
    if not levels.load_level(MAP_PATH):
        raise RuntimeError("Could not load the comparison map.")
else:
    if not levels.new_level(MAP_PATH):
        raise RuntimeError("Could not create the comparison map.")

actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
comparisons = [a for a in actors.get_all_level_actors() if a.get_class() == comparison_class]
if len(comparisons) > 1:
    raise RuntimeError("Multiple comparison actors found; refusing to modify an ambiguous map.")
actor = comparisons[0] if comparisons else actors.spawn_actor_from_class(
    comparison_class, unreal.Vector(0, 0, 200), unreal.Rotator(0, 0, 0))
actor.set_actor_label("Cloth comparison - CUSTOM cyan / CHAOS orange")

results = []


def check_report(case, report, stationary=False):
    if "finite=yes" not in report:
        raise AssertionError(f"{case}: non-finite simulation: {report}")
    if "pin drift 0.000000 / 0.000000 cm" not in report:
        raise AssertionError(f"{case}: a pinned particle moved: {report}")
    if "edges=705 hinges=645" not in report:
        raise AssertionError(f"{case}: unexpected topology: {report}")
    numbers = re.search(r"RMS=([\d.]+) cm max=([\d.]+) cm", report)
    if not numbers:
        raise AssertionError(f"{case}: missing difference metrics: {report}")
    if stationary and (float(numbers[1]) != 0 or "max speed 0.00 / 0.00 cm/s" not in report):
        raise AssertionError(f"{case}: resting cloth moved: {report}")


def run_case(name, steps, gravity, push=False, gust=False):
    actor.set_editor_property("gravity_enabled", gravity)
    actor.set_editor_property("oscillating_force", gust)
    actor.reset_comparison()
    initial = actor.get_comparison_report()
    check_report(name, initial, stationary=True)
    if push:
        actor.push_both()
    samples = []
    for step in range(steps):
        actor.step_once()
        if (step + 1) % 60 == 0 or step + 1 == steps:
            report = actor.get_comparison_report()
            check_report(name, report, stationary=not (gravity or push or gust))
            samples.append(report)
    results.append({"case": name, "steps": steps, "samples": samples})
    unreal.log("[PBDComparisonTest] " + name + ": " + samples[-1])


actor.set_editor_property("num_x", 16)
actor.set_editor_property("num_y", 16)
actor.set_editor_property("spacing", 10.0)
actor.set_editor_property("fixed_delta_time", 1.0 / 60.0)
actor.set_editor_property("solver_iterations", 8)
actor.set_editor_property("paused", False)
actor.set_editor_property("gravity", unreal.Vector(0, 0, -980))
actor.set_editor_property("push_velocity", unreal.Vector(0, 150, 0))
actor.set_editor_property("force_acceleration", unreal.Vector(0, 150, 0))
run_case("rest_without_gravity", 60, False)
run_case("gravity_only", 300, True)
run_case("out_of_plane_velocity", 120, False, push=True)
run_case("gravity_and_periodic_force", 180, True, gust=True)

# Exercise all three real constraints through the same launcher used by mouse input.
actor.set_editor_property("fixed_delta_time", 1.0 / 120.0)
actor.set_editor_property("solver_iterations", 16)
for shape, name in enumerate(("sphere", "cylinder", "capsule")):
    actor.set_editor_property("gravity_enabled", False)
    actor.set_editor_property("oscillating_force", False)
    actor.reset_comparison()
    actor.fire_at_local_target(unreal.Vector(0, 0, -15), shape)
    maximum_speed = 0.0
    for step in range(360):
        actor.step_once()
        report = actor.get_comparison_report()
        check_report(name, report)
        penetration = re.search(r"penetration=([\d.]+) / ([\d.]+)", report)
        if not penetration or max(map(float, penetration.groups())) > 0.001:
            raise AssertionError(f"{name}: residual penetration: {report}")
        speed = re.search(r"max speed ([\d.]+) / ([\d.]+)", report)
        maximum_speed = max(maximum_speed, min(map(float, speed.groups())))
    if maximum_speed < 1.0:
        raise AssertionError(f"{name}: projectile never moved both cloths")
    results.append({"case": "launcher_" + name, "steps": 360, "report": report,
                    "maximum_shared_speed": maximum_speed})

# A deterministic random sequence, pause/step/reset and shot expiration use simulation time.
actor.reset_comparison()
actor.fire_at_local_target(unreal.Vector(0, 0, 0))
first = actor.get_comparison_report().split("last=")[1]
actor.reset_comparison()
actor.fire_at_local_target(unreal.Vector(0, 0, 0))
assert actor.get_comparison_report().split("last=")[1] == first
for _ in range(740):
    actor.step_once()
assert "active=0" in actor.get_comparison_report()

# Idempotently furnish this existing test map. Decoration has no physics role.
def scene_actor(cls, label, location, rotation=unreal.Rotator(0, 0, 0)):
    found = [a for a in actors.get_all_level_actors() if a.get_actor_label() == label]
    obj = found[0] if found else actors.spawn_actor_from_class(cls, unreal.Vector(*location), rotation)
    obj.set_actor_label(label)
    obj.set_actor_location(unreal.Vector(*location), False, False)
    obj.set_actor_rotation(rotation, False)
    return obj

cube = unreal.load_asset("/Engine/BasicShapes/Cube.Cube")
def matte_material(name, color):
    path = "/Game/Tests/Materials/" + name
    material = unreal.load_asset(path)
    if material is None:
        material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
            name, "/Game/Tests/Materials", unreal.Material, unreal.MaterialFactoryNew())
        tint = unreal.MaterialEditingLibrary.create_material_expression(material, unreal.MaterialExpressionConstant3Vector)
        tint.set_editor_property("constant", unreal.LinearColor(*color, 1))
        unreal.MaterialEditingLibrary.connect_material_property(tint, "", unreal.MaterialProperty.MP_BASE_COLOR)
        roughness = unreal.MaterialEditingLibrary.create_material_expression(material, unreal.MaterialExpressionConstant)
        roughness.set_editor_property("r", 0.8)
        unreal.MaterialEditingLibrary.connect_material_property(roughness, "", unreal.MaterialProperty.MP_ROUGHNESS)
        unreal.MaterialEditingLibrary.recompile_material(material)
        unreal.EditorAssetLibrary.save_loaded_asset(material)
    if name == "M_PBD_Projectile":
        unreal.MaterialEditingLibrary.set_base_material_usage(material, unreal.MaterialUsage.MATUSAGE_INSTANCED_STATIC_MESHES, True)
        unreal.MaterialEditingLibrary.recompile_material(material)
        unreal.EditorAssetLibrary.save_loaded_asset(material)
    return material

wall_material = matte_material("M_PBD_Backdrop", (0.025, 0.04, 0.065))
floor_material = matte_material("M_PBD_Floor", (0.16, 0.19, 0.23))
frame_material = matte_material("M_PBD_Frame", (0.06, 0.085, 0.11))
shot_material = matte_material("M_PBD_Projectile", (0.7, 0.28, 0.035))
actor.get_editor_property("projectile_spheres").set_material(0, shot_material)
actor.get_editor_property("projectile_cylinders").set_material(0, shot_material)

def block(label, location, scale, material=frame_material):
    obj = scene_actor(unreal.StaticMeshActor, label, location)
    component = obj.static_mesh_component
    component.set_static_mesh(cube)
    component.set_material(0, material)
    component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    obj.set_actor_scale3d(unreal.Vector(*scale))
    return obj

block("Comparison stage floor", (0, 0, 30), (12, 12, 0.2), floor_material)
block("Comparison backdrop", (0, 240, 270), (12, 0.2, 5), wall_material)
for side in (-1, 1):
    block(f"Comparison pedestal {side}", (side * 115, 0, 55), (2, 1.8, 0.3))
    for edge in (-1, 1):
        block(f"Comparison frame {side} {edge}", (side * 115 + edge * 85, 35, 180), (0.045, 0.045, 2.6))
    block(f"Comparison top rail {side}", (side * 115, 35, 310), (1.75, 0.045, 0.045))

sun = scene_actor(unreal.DirectionalLight, "Comparison key light", (0, -200, 500), unreal.Rotator(pitch=-40, yaw=65, roll=0))
sun.light_component.set_intensity(2.0)
fill = scene_actor(unreal.PointLight, "Comparison fill light", (0, -200, 350))
fill.point_light_component.set_intensity(800.0)
fill.point_light_component.set_attenuation_radius(1800.0)
sky = scene_actor(unreal.SkyLight, "Comparison sky light", (0, 0, 500))
sky.light_component.set_intensity(1.0)
scene_actor(unreal.SkyAtmosphere, "Comparison atmosphere", (0, 0, 0))

post = scene_actor(unreal.PostProcessVolume, "Comparison exposure", (0, 0, 0))
post.set_editor_property("unbound", True)
settings = post.get_editor_property("settings")
settings.set_editor_property("override_auto_exposure_min_brightness", True)
settings.set_editor_property("override_auto_exposure_max_brightness", True)
settings.set_editor_property("auto_exposure_min_brightness", 1.0)
settings.set_editor_property("auto_exposure_max_brightness", 1.0)
post.set_editor_property("settings", settings)

actor.set_editor_property("gravity_enabled", True)
actor.set_editor_property("oscillating_force", False)
actor.set_editor_property("paused", False)
actor.set_editor_property("overlay", False)
actor.set_editor_property("fixed_delta_time", 1.0 / 120.0)
actor.set_editor_property("solver_iterations", 16)
actor.reset_comparison()

# A useful editor viewport; the actor's own camera is selected automatically in PIE.
unreal.EditorLevelLibrary.set_level_viewport_camera_info(
    unreal.Vector(0, -650, 220), unreal.Rotator(pitch=0, yaw=90, roll=0))
if not levels.save_current_level():
    raise RuntimeError("Comparison map save failed.")
report_path = Path(unreal.Paths.project_saved_dir()) / "Tests" / "PBDChaosComparison.json"
report_path.parent.mkdir(parents=True, exist_ok=True)
report_path.write_text(json.dumps(results, indent=2), encoding="utf-8")
unreal.log("[PBDComparisonTest] PASS. Map=" + MAP_PATH + " Report=" + str(report_path))
