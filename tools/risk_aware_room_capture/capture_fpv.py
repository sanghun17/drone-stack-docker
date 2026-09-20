"""Capture two approximate FPV views from the existing furnished room map."""
import json
import math
import os
import subprocess
import time
import traceback
from pathlib import Path

import unreal

ROOT = Path('/home/ml/drone-stack-docker')
OUT = ROOT / 'paper_assets/risk_aware_room_actual'
MAP = '/Game/RiskAwareRoomFigure/Room'
WIDTH = int(os.environ.get('ROOM_FPV_WIDTH', '1920'))
HEIGHT = int(os.environ.get('ROOM_FPV_HEIGHT', '1080'))
SMOKE_PNG = OUT / os.environ.get('ROOM_FPV_SMOKE_NAME', 'risk_aware_room_fpv_smoke.png')
CLEAR_PNG = OUT / os.environ.get('ROOM_FPV_CLEAR_NAME', 'risk_aware_room_fpv_clear.png')
ELL = unreal.EditorLevelLibrary

# Approximate the two green frustums in the supplied paper sketch. UE X points
# toward image-top in the overhead figure and UE Y toward image-right.
POSES = [
    {
        'name': 'smoke',
        'output': SMOKE_PNG,
        'location': (760.0, 620.0, 145.0),
        'target': (1060.0, 350.0, 155.0),
        'horizontal_fov_deg': 82.0,
        'description': 'closer middle-right flight position, looking directly into smoke',
    },
    {
        'name': 'clear',
        'output': CLEAR_PNG,
        'location': (800.0, 700.0, 145.0),
        'target': (2200.0, 900.0, 0.0),
        'horizontal_fov_deg': 82.0,
        'description': 'middle flight position, looking through the extended upper-right corridor',
    },
]


def find_actor(label):
    for actor in ELL.get_all_level_actors():
        if actor.get_actor_label() == label:
            return actor
    raise RuntimeError('Actor not found: ' + label)


def look_rotation(location, target):
    dx = target[0] - location[0]
    dy = target[1] - location[1]
    dz = target[2] - location[2]
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    return unreal.Rotator(pitch=pitch, yaw=yaw, roll=0.0)


def apply_pose(camera, pose):
    camera.set_actor_location(unreal.Vector(*pose['location']), False, False)
    camera.set_actor_rotation(look_rotation(pose['location'], pose['target']), False)
    component = camera.get_component_by_class(unreal.CameraComponent)
    component.set_editor_property('projection_mode', unreal.CameraProjectionMode.PERSPECTIVE)
    component.set_editor_property('field_of_view', pose['horizontal_fov_deg'])
    component.set_editor_property('aspect_ratio', float(WIDTH) / HEIGHT)
    component.set_editor_property('constrain_aspect_ratio', True)
    ELL.set_level_viewport_camera_info(camera.get_actor_location(), camera.get_actor_rotation())
    ELL.pilot_level_actor(camera)
    ELL.editor_set_game_view(True)
    ELL.editor_invalidate_viewports()


def set_box_transform(actor, centre, size):
    actor.set_actor_location(unreal.Vector(*centre), False, False)
    actor.set_actor_scale3d(unreal.Vector(*(value / 100.0 for value in size)))


def add_fpv_enclosure():
    """Temporarily hide the roofless-map void without altering the saved map."""
    wall_height = 650.0
    for label in [
            'Figure_LeftWall', 'Figure_RightWall', 'Figure_BottomWall',
            'Figure_TopLeftWall', 'Figure_L_Vertical', 'Figure_L_Horizontal']:
        actor = find_actor(label)
        location = actor.get_actor_location()
        scale = actor.get_actor_scale3d()
        actor.set_actor_location(
            unreal.Vector(location.x, location.y, wall_height / 2.0), False, False)
        actor.set_actor_scale3d(unreal.Vector(scale.x, scale.y, wall_height / 100.0))

    floor_actor = find_actor('Figure_ExitFloor')
    set_box_transform(floor_actor, (1900.0, 900.0, -5.0), (800.0, 600.0, 10.0))
    floor_material = find_actor('Figure_Floor').static_mesh_component.get_material(0)
    wall_material = find_actor('Figure_RightWall').static_mesh_component.get_material(0)

    additions = [
        ('FPV_CorridorLeftWall', (1900.0, 587.5, wall_height / 2.0),
         (800.0, 25.0, wall_height), wall_material),
        ('FPV_CorridorRightWall', (1900.0, 1212.5, wall_height / 2.0),
         (800.0, 25.0, wall_height), wall_material),
        ('FPV_CorridorEndWall', (2312.5, 900.0, wall_height / 2.0),
         (25.0, 625.0, wall_height), wall_material),
    ]
    for label, centre, size, material in additions:
        actor = ELL.spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(*centre))
        if actor is None:
            raise RuntimeError('Could not spawn ' + label)
        actor.set_actor_label(label)
        actor.static_mesh_component.set_static_mesh(unreal.load_asset('/Engine/BasicShapes/Cube'))
        actor.static_mesh_component.set_material(0, material)
        set_box_transform(actor, centre, size)

    return wall_height


def activate_editor_window():
    """Make UE's native viewport service HighResShot immediately on Linux."""
    try:
        result = subprocess.check_output([
            'xdotool', 'search', '--pid', str(os.getpid()),
            '--name', 'RiskAwareRoomCapture - Unreal Editor'])
        window_ids = result.decode().strip().splitlines()
        if window_ids:
            subprocess.call(['xdotool', 'windowactivate', window_ids[-1]])
    except Exception as exc:
        unreal.log_warning('Could not activate UE4 window: ' + str(exc))


try:
    OUT.mkdir(parents=True, exist_ok=True)
    for pose in POSES:
        if pose['output'].exists():
            raise RuntimeError('Output already exists: ' + str(pose['output']))

    ELL.load_level(MAP)
    wall_height = add_fpv_enclosure()
    camera = find_actor('Figure_Camera')
    smoke_actor = find_actor('Figure_Smoke')
    smoke = smoke_actor.get_component_by_class(unreal.NiagaraComponent)
    # Lighter, sparser smoke lets the wall behind remain perceptible in FPV.
    smoke.set_niagara_variable_float('User.SpawnRate', 9.0)
    smoke.set_niagara_variable_float('User.ScaleColor', 0.8)
    smoke.set_niagara_variable_linear_color(
        'User.Color', unreal.LinearColor(1.0, 1.0, 1.0, 0.45))
    smoke.deactivate()
    smoke.activate(True)
    ELL.set_selected_level_actors([])

    world = ELL.get_editor_world()
    for command in [
            'r.SSR.Quality 0', 'r.ReflectionEnvironment 0', 'r.BloomQuality 0',
            'r.MotionBlurQuality 0', 'r.ScreenPercentage 100',
            'r.TextureStreaming 1', 'r.Streaming.PoolSize 4096',
            'r.Streaming.FullyLoadUsedTextures 1', 'r.Streaming.UseAllMips 1',
            'r.Streaming.MipBias 0', 'r.MipMapLODBias 0']:
        unreal.SystemLibrary.execute_console_command(world, command)

    metadata = {
        'renderer': 'Unreal Engine 4.27 editor Vulkan, native viewport HighResShot',
        'map': MAP,
        'resolution': [WIDTH, HEIGHT],
        'coordinate_convention': 'UE +X = overhead image top; UE +Y = overhead image right',
        'temporary_fpv_environment': {
            'wall_height_cm': wall_height,
            'corridor_x_range_cm': [1500.0, 2300.0],
            'corridor_y_range_cm': [600.0, 1200.0],
            'saved_map_modified': False,
        },
        'temporary_smoke_parameters': {
            'SpawnRate': 9.0, 'ScaleColor': 0.8, 'ColorAlpha': 0.45,
        },
        'poses': [],
    }
    for pose in POSES:
        rotation = look_rotation(pose['location'], pose['target'])
        metadata['poses'].append({
            key: (str(value) if isinstance(value, Path) else value)
            for key, value in pose.items()
        })
        metadata['poses'][-1]['rotation_deg'] = {
            'pitch': rotation.pitch, 'yaw': rotation.yaw, 'roll': rotation.roll}
    (OUT / 'risk_aware_room_fpv_metadata.json').write_text(json.dumps(metadata, indent=2))

    started = time.monotonic()
    state = {'pose_index': -1, 'requested_at': None, 'settle_until': None}

    def tick(delta):
        try:
            elapsed = time.monotonic() - started
            if state['pose_index'] < 0:
                # Match the overhead capture's Niagara warm-up before freezing it,
                # so both FPVs show the same particle state.
                smoke.advance_simulation(1, 1.0 / 60.0)
                ELL.editor_invalidate_viewports()
                if elapsed < 25.0:
                    return
                smoke.set_paused(True)
                state['pose_index'] = 0
                apply_pose(camera, POSES[0])
                state['settle_until'] = elapsed + 1.0
                return

            pose = POSES[state['pose_index']]
            if state['requested_at'] is None:
                if elapsed < state['settle_until']:
                    return
                activate_editor_window()
                unreal.SystemLibrary.execute_console_command(
                    world,
                    'HighResShot {}x{} filename={}'.format(
                        WIDTH, HEIGHT, pose['output']))
                state['requested_at'] = elapsed
                unreal.log('ROOM_FPV_REQUESTED {} {}'.format(pose['name'], pose['output']))
                return

            if not pose['output'].exists():
                if elapsed - state['requested_at'] > 180.0:
                    raise RuntimeError('Timed out waiting for FPV: ' + pose['name'])
                return
            if pose['output'].stat().st_size < 1000:
                raise RuntimeError('FPV PNG is unexpectedly small: ' + str(pose['output']))

            unreal.log('ROOM_FPV_DONE {} {}'.format(pose['name'], pose['output']))
            state['pose_index'] += 1
            state['requested_at'] = None
            if state['pose_index'] >= len(POSES):
                unreal.unregister_slate_post_tick_callback(handle)
                unreal.SystemLibrary.quit_editor()
                return
            apply_pose(camera, POSES[state['pose_index']])
            state['settle_until'] = elapsed + 1.0
        except Exception:
            (OUT / 'fpv_capture_error.txt').write_text(traceback.format_exc())
            unreal.log_error(traceback.format_exc())
            unreal.unregister_slate_post_tick_callback(handle)
            unreal.SystemLibrary.quit_editor()

    handle = unreal.register_slate_post_tick_callback(tick)
except Exception:
    (OUT / 'fpv_capture_error.txt').write_text(traceback.format_exc())
    unreal.log_error(traceback.format_exc())
    unreal.SystemLibrary.quit_editor()
