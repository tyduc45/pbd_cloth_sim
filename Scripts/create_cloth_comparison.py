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

actor.set_editor_property("gravity_enabled", True)
actor.set_editor_property("oscillating_force", False)
actor.set_editor_property("paused", False)
actor.set_editor_property("overlay", False)
actor.reset_comparison()

# A useful editor viewport; the actor's own camera is selected automatically in PIE.
unreal.EditorLevelLibrary.set_level_viewport_camera_info(
    unreal.Vector(0, -650, 220), unreal.Rotator(0, 90, 0))
if not levels.save_current_level():
    raise RuntimeError("Comparison map save failed.")
report_path = Path(unreal.Paths.project_saved_dir()) / "Tests" / "PBDChaosComparison.json"
report_path.parent.mkdir(parents=True, exist_ok=True)
report_path.write_text(json.dumps(results, indent=2), encoding="utf-8")
unreal.log("[PBDComparisonTest] PASS. Map=" + MAP_PATH + " Report=" + str(report_path))
