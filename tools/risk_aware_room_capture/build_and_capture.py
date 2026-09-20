"""Run in the full UE4 editor, with copied risk-aware assets in /Game.

Camera points exactly down, but uses perspective projection. No image synthesis
or postprocessing outside Unreal is used for the resulting PNG.
"""
import json
import os
import time
import traceback
from pathlib import Path

import unreal

ROOT = Path('/home/ml/drone-stack-docker')
OUT = ROOT / 'paper_assets/risk_aware_room_actual'
ASSETS = '/Game/RiskAwareRoomFigure'
MAP = ASSETS + '/Room'
WIDTH = int(os.environ.get('ROOM_CAPTURE_WIDTH', '1024'))
HEIGHT = int(os.environ.get('ROOM_CAPTURE_HEIGHT', '1280'))
PNG = OUT / os.environ.get('ROOM_CAPTURE_NAME', 'preview.png')
VIEWPORT_CAPTURE = os.environ.get('ROOM_CAPTURE_VIEWPORT', '1') == '1'
SMOKE_ASSET = '/Game/SmokePackage/Fx/Niagara/NS_Fx_Smoke02_Rise'
SOFA_SCALE = 1.35
ELL = unreal.EditorLevelLibrary
MEL = unreal.MaterialEditingLibrary
EAL = unreal.EditorAssetLibrary


def constant(material, value, prop):
    expression = MEL.create_material_expression(material, unreal.MaterialExpressionConstant)
    expression.set_editor_property('r', value)
    MEL.connect_material_property(expression, '', prop)


def duplicate_matte(source, name):
    dest = ASSETS + '/' + name
    if EAL.does_asset_exist(dest):
        return unreal.load_asset(dest)
    mat = EAL.duplicate_asset(source, dest)
    constant(mat, 1.0, unreal.MaterialProperty.MP_ROUGHNESS)
    constant(mat, 0.0, unreal.MaterialProperty.MP_SPECULAR)
    constant(mat, 0.0, unreal.MaterialProperty.MP_METALLIC)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    return mat


def original_wood_material():
    path = ASSETS + '/M_OriginalWalnutMatte'
    if EAL.does_asset_exist(path):
        return unreal.load_asset(path)
    mat = unreal.AssetToolsHelpers.get_asset_tools().create_asset('M_OriginalWalnutMatte', ASSETS, unreal.Material, unreal.MaterialFactoryNew())
    uv = MEL.create_material_expression(mat, unreal.MaterialExpressionTextureCoordinate)
    uv.set_editor_property('u_tiling', 1.25)
    uv.set_editor_property('v_tiling', 1.5)
    tex = MEL.create_material_expression(mat, unreal.MaterialExpressionTextureSample)
    tex.set_editor_property('texture', unreal.load_asset('/Game/StarterContent/Textures/T_Wood_Floor_Walnut_D'))
    MEL.connect_material_expressions(uv, '', tex, 'UVs')
    MEL.connect_material_property(tex, 'RGB', unreal.MaterialProperty.MP_BASE_COLOR)
    constant(mat, 1.0, unreal.MaterialProperty.MP_ROUGHNESS)
    constant(mat, 0.0, unreal.MaterialProperty.MP_SPECULAR)
    MEL.recompile_material(mat)
    EAL.save_loaded_asset(mat)
    return mat


def spawn(cls, name, xyz, rotation=(0, 0, 0)):
    actor = ELL.spawn_actor_from_class(cls, unreal.Vector(*xyz), unreal.Rotator(pitch=rotation[0], yaw=rotation[1], roll=rotation[2]))
    if actor is None:
        raise RuntimeError('Could not spawn ' + name)
    actor.set_actor_label(name)
    return actor


def box(name, centre, size, material):
    a = spawn(unreal.StaticMeshActor, name, centre)
    a.set_actor_scale3d(unreal.Vector(*(v / 100.0 for v in size)))
    c = a.static_mesh_component
    c.set_static_mesh(unreal.load_asset('/Engine/BasicShapes/Cube'))
    c.set_material(0, material)
    c.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
    return a


def static_mesh(name, asset_path, rotation=(0, 0, 0), scale=(1, 1, 1)):
    a = spawn(unreal.StaticMeshActor, name, (0, 0, 0), rotation)
    a.set_actor_scale3d(unreal.Vector(*scale))
    c = a.static_mesh_component
    c.set_static_mesh(unreal.load_asset(asset_path))
    c.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
    return a


def move_bounds_centre(actor, target):
    centre, _ = actor.get_actor_bounds(False)
    delta = unreal.Vector(target[0] - centre.x, target[1] - centre.y, target[2] - centre.z)
    actor.set_actor_location(actor.get_actor_location() + delta, False, False)
    return delta


def add_furniture():
    # Existing modular sofa, scaled to about 4.13 m wide to nearly match the
    # 4.35 m partition. Its back is flush with the main-room face of that
    # partition and it faces image-up (+X).
    sofa_parts = []
    for suffix in ['SM_Sofa', 'SM_SofaBack', 'SM_SofaBase', 'SM_SofaSeats']:
        sofa_parts.append(static_mesh(
            'Furniture_' + suffix,
            '/Game/ModernLivingRoom/StaticMesh/Furniture/' + suffix,
            rotation=(0, 180, 0), scale=(SOFA_SCALE, SOFA_SCALE, SOFA_SCALE)))
    sofa_centre, sofa_extent = sofa_parts[0].get_actor_bounds(False)
    sofa_target = (517.5 + sofa_extent.x, 1000.0, sofa_extent.z)
    sofa_delta = unreal.Vector(
        sofa_target[0] - sofa_centre.x,
        sofa_target[1] - sofa_centre.y,
        sofa_target[2] - sofa_centre.z)
    for actor in sofa_parts:
        actor.set_actor_location(actor.get_actor_location() + sofa_delta, False, False)

    # Top frame remains fixed. The next two centres are 200 cm apart (2x gap).
    painting_assets = ['SM_Painting1', 'SM_Painting2', 'SM_Painting1']
    painting_x = [1375.0, 1175.0, 975.0]
    for index, (asset, x_pos) in enumerate(zip(painting_assets, painting_x), 1):
        actor = static_mesh(
            'Furniture_Painting_{:02d}'.format(index),
            '/Game/ModernLivingRoom/StaticMesh/Furniture/' + asset,
            rotation=(0, 90, 0), scale=(1.35, 1.35, 1.35))
        _, extent = actor.get_actor_bounds(False)
        move_bounds_centre(actor, (x_pos, 1200.0 - extent.y, 150.0))

    return sofa_parts, painting_x


def setup():
    OUT.mkdir(parents=True, exist_ok=True)
    if (ROOT / '.build/risk-aware-room-capture/Content/RiskAwareRoomFigure/Room.umap').exists():
        ELL.load_level(MAP)
        for a in ELL.get_all_level_actors():
            if a.get_actor_label().startswith(('Figure_', 'Furniture_')):
                ELL.destroy_actor(a)
    else:
        ELL.new_level(MAP)

    for path in ['/Game/StarterContent/Textures/T_Wood_Floor_Walnut_D',
                 '/Game/SmokePackage/Fx/Texture/T_Fx_SeqSmoke02',
                 '/Game/SmokePackage/Common/Material/DefaultTexture/T_Fx_C',
                 '/Game/SmokePackage/Common/Material/DefaultTexture/T_Fx_LC']:
        texture = unreal.load_asset(path)
        texture.set_editor_property('never_stream', True)
        texture.set_editor_property('lod_bias', 0)
        EAL.save_loaded_asset(texture)
    floor = original_wood_material()
    wall = duplicate_matte('/Game/StarterContent/Materials/M_Basic_Wall', 'M_Wall_Matte')
    # UE X points toward the image top, UE Y toward image right. Units: cm.
    box('Figure_Floor', (750, 600, -5), (1500, 1200, 10), floor)
    box('Figure_ExitFloor', (1575, 900, -5), (150, 600, 10), floor)
    box('Figure_LeftWall', (750, -12.5, 125), (1550, 25, 250), wall)
    box('Figure_RightWall', (750, 1212.5, 125), (1550, 25, 250), wall)
    box('Figure_BottomWall', (-12.5, 600, 125), (25, 1200, 250), wall)
    box('Figure_TopLeftWall', (1512.5, 287.5, 125), (25, 625, 250), wall)
    box('Figure_L_Vertical', (250, 800, 125), (500, 35, 250), wall)
    box('Figure_L_Horizontal', (500, 1000, 125), (35, 435, 250), wall)
    sofa_parts, painting_x = add_furniture()

    # Keep all furniture textures at their authored resolution for the 4K capture.
    for texture_dir in ['Sofa', 'SofaBack', 'SofaBase', 'SofaSeats', 'Paintings', 'Painting2']:
        for asset_path in EAL.list_assets('/Game/ModernLivingRoom/Textures/' + texture_dir, recursive=True, include_folder=False):
            texture = unreal.load_asset(asset_path)
            if isinstance(texture, unreal.Texture2D):
                texture.set_editor_property('never_stream', True)
                texture.set_editor_property('lod_bias', 0)

    # Illumination actors have no visible fixtures or geometry.
    for name, rot, power, shadows in [
        ('Key', (-65, -35, 0), 12.0, False),
        ('Fill', (-55, 145, 0), 6.0, False),
    ]:
        light = spawn(unreal.DirectionalLight, 'Figure_' + name, (750, 600, 1400), rot)
        lc = light.get_component_by_class(unreal.DirectionalLightComponent)
        lc.set_mobility(unreal.ComponentMobility.MOVABLE)
        lc.set_editor_property('intensity', power)
        lc.set_editor_property('cast_shadows', shadows)
        lc.set_editor_property('light_source_angle', 8.0)

    sky = spawn(unreal.SkyLight, 'Figure_Ambient', (750, 600, 1000))
    sk = sky.get_component_by_class(unreal.SkyLightComponent)
    sk.set_mobility(unreal.ComponentMobility.MOVABLE)
    sk.set_editor_property('source_type', unreal.SkyLightSourceType.SLS_SPECIFIED_CUBEMAP)
    sk.set_editor_property('cubemap', unreal.load_asset('/Engine/MapTemplates/Sky/DaylightAmbientCubemap'))
    sk.set_intensity(0.8)
    sk.recapture_sky()

    camera = spawn(unreal.CameraActor, 'Figure_Camera', (780, 600, 1800), (-90, 0, 0))
    cc = camera.get_component_by_class(unreal.CameraComponent)
    cc.set_editor_property('projection_mode', unreal.CameraProjectionMode.PERSPECTIVE)
    cc.set_editor_property('field_of_view', 50.0)
    cc.set_editor_property('aspect_ratio', float(WIDTH) / HEIGHT)
    cc.set_editor_property('constrain_aspect_ratio', True)
    post = cc.get_editor_property('post_process_settings')
    for name, value in [('auto_exposure_min_brightness', 1.0),
                        ('auto_exposure_max_brightness', 1.0),
                        ('auto_exposure_bias', 0.0),
                        ('motion_blur_amount', 0.0),
                        ('bloom_intensity', 0.0),
                        ('screen_space_reflection_intensity', 0.0)]:
        post.set_editor_property('override_' + name, True)
        post.set_editor_property(name, value)
    cc.set_editor_property('post_process_settings', post)
    cc.set_editor_property('post_process_blend_weight', 1.0)

    capture = spawn(unreal.SceneCapture2D, 'Figure_RenderCamera', (780, 600, 1800), (-90, 0, 0))
    sc = capture.get_component_by_class(unreal.SceneCaptureComponent2D)
    target = unreal.RenderingLibrary.create_render_target2d(ELL.get_editor_world(), WIDTH, HEIGHT, unreal.TextureRenderTargetFormat.RTF_RGBA8)
    sc.set_editor_property('texture_target', target)
    sc.set_editor_property('capture_source', unreal.SceneCaptureSource.SCS_FINAL_COLOR_LDR)
    sc.set_editor_property('projection_type', unreal.CameraProjectionMode.PERSPECTIVE)
    sc.set_editor_property('fov_angle', 50.0)
    sc.set_editor_property('post_process_settings', post)
    sc.set_editor_property('post_process_blend_weight', 1.0)
    sc.set_editor_property('capture_every_frame', True)
    sc.set_editor_property('show_flag_settings', [
        unreal.EngineShowFlagsSetting(show_flag_name='BillboardSprites', enabled=False),
        unreal.EngineShowFlagsSetting(show_flag_name='ModeWidgets', enabled=False),
    ])
    for a in ELL.get_all_level_actors():
        if isinstance(a, (unreal.DirectionalLight, unreal.SkyLight,
                          unreal.CameraActor, unreal.SceneCapture2D)):
            sc.hide_actor_components(a)

    smoke = spawn(unreal.NiagaraActor, 'Figure_Smoke', (1060, 350, 35))
    nc = smoke.get_component_by_class(unreal.NiagaraComponent)
    nc.set_asset(unreal.load_asset(SMOKE_ASSET))
    # Same original system; explicit figure parameters constrain its footprint.
    nc.set_niagara_variable_float('User.SmokeSize', 4.0)
    nc.set_niagara_variable_float('User.GroundRange', 1.2)
    nc.set_niagara_variable_float('User.SpawnRate', 20.0)
    nc.set_niagara_variable_float('User.LifeTime', 2.0)
    nc.set_niagara_variable_float('User.InitialVelocity', 0.15)
    nc.set_niagara_variable_float('User.WindStrength', 0.0)
    nc.set_niagara_variable_float('User.ScaleColor', 2.0)
    nc.set_niagara_variable_linear_color('User.Color', unreal.LinearColor(1, 1, 1, 1))
    nc.activate(True)

    world = ELL.get_editor_world()
    for command in ['r.SSR.Quality 0', 'r.ReflectionEnvironment 0',
                    'r.BloomQuality 0', 'r.MotionBlurQuality 0',
                    'r.ScreenPercentage 100', 'r.TextureStreaming 1',
                    'r.Streaming.PoolSize 4096', 'r.Streaming.FullyLoadUsedTextures 1',
                    'r.Streaming.UseAllMips 1', 'r.Streaming.MipBias 0', 'r.MipMapLODBias 0',
                    'r.Editor.ConsistentAllFrameRate 1']:
        unreal.SystemLibrary.execute_console_command(world, command)
    ELL.set_selected_level_actors([])
    ELL.set_level_viewport_camera_info(camera.get_actor_location(), camera.get_actor_rotation())
    ELL.pilot_level_actor(camera)
    ELL.editor_set_game_view(True)
    if VIEWPORT_CAPTURE:
        sc.set_editor_property('capture_every_frame', False)
    # The render target is transient; keep the saved map independent of it.
    sc.set_editor_property('texture_target', None)
    if not ELL.save_current_level():
        raise RuntimeError('Could not save the room map')
    sc.set_editor_property('texture_target', target)
    manifest = {
        'renderer': 'Unreal Engine 4.27 editor Vulkan, actual scene render',
        'capture_method': ('UE4 native viewport high-resolution screenshot' if VIEWPORT_CAPTURE
                           else 'SceneCapture2D render target (diagnostic only)'),
        'map': MAP, 'room_size_m': [12, 15], 'wall_height_m': 2.5,
        'wall_thickness_m': 0.25, 'partition_thickness_m': 0.35,
        'camera_projection': 'perspective', 'camera_pitch_deg': -90,
        'camera_location_ue_cm': [780, 600, 1800], 'horizontal_fov_deg': 50,
        'actual_camera_rotation': str(camera.get_actor_rotation()),
        'actual_camera_forward': str(camera.get_actor_forward_vector()),
        'resolution': [WIDTH, HEIGHT], 'floor_source': '/Game/StarterContent/Textures/T_Wood_Floor_Walnut_D',
        'wall_source': '/Game/StarterContent/Materials/M_Basic_Wall', 'smoke_source': SMOKE_ASSET,
        'floor_roughness': 1.0, 'floor_specular': 0.0,
        'smoke_location_ue_cm': [1060, 350, 35],
        'smoke_parameters': {'SmokeSize': 4.0, 'GroundRange': 1.2,
            'SpawnRate': 20.0, 'LifeTime': 2.0, 'InitialVelocity': 0.15,
            'WindStrength': 0.0, 'ScaleColor': 2.0},
        'furniture': {
            'sofa_assets': [a.get_actor_label() for a in sofa_parts],
            'sofa_scale': SOFA_SCALE,
            'sofa_partition_outer_face_x_cm': 517.5,
            'sofa_facing': '+X / image top / open exit',
            'painting_centres_x_cm': painting_x,
            'painting_gap_cm': 200.0,
            'painting_wall_inner_face_y_cm': 1200.0,
        },
        'actors': [{'label': a.get_actor_label(), 'class': a.get_class().get_name(),
                    'location': str(a.get_actor_location()), 'rotation': str(a.get_actor_rotation()),
                    'bounds': str(a.get_actor_bounds(False)), 'scale': str(a.get_actor_scale3d())}
                   for a in ELL.get_all_level_actors()],
        'output': str(PNG),
    }
    (OUT / (PNG.stem + '_metadata.json')).write_text(json.dumps(manifest, indent=2))
    return camera, nc, sc, target


try:
    CAMERA, SMOKE, CAPTURE, TARGET = setup()
    STARTED = time.monotonic()
    STATE = {'shot': False, 'idle': False}
    LIVE_COMMAND = ROOT / '.build/risk-aware-room-capture/live_command.py'

    def tick(delta):
        try:
            if STATE['idle']:
                if LIVE_COMMAND.exists():
                    command = LIVE_COMMAND.read_text()
                    LIVE_COMMAND.rename(LIVE_COMMAND.with_suffix('.executed.py'))
                    try:
                        exec(compile(command, str(LIVE_COMMAND), 'exec'), globals())
                    except Exception:
                        (OUT / 'live_command_error.txt').write_text(traceback.format_exc())
                        unreal.log_error(traceback.format_exc())
                return
            elapsed = time.monotonic() - STARTED
            if not STATE['shot']:
                # Explicitly advance particles in editor, not only in PIE.
                SMOKE.advance_simulation(1, 1.0 / 60.0)
                ELL.editor_invalidate_viewports()
                if elapsed > 25:
                    SMOKE.set_paused(True)
                    if VIEWPORT_CAPTURE:
                        # The viewport is already piloted to Figure_Camera above. Invoke
                        # the editor's native HighResShot path directly; the automation
                        # wrapper can wait forever for its task callback on UE 4.27/Linux.
                        unreal.SystemLibrary.execute_console_command(
                            ELL.get_editor_world(),
                            'HighResShot {}x{} filename={}'.format(WIDTH, HEIGHT, PNG))
                    else:
                        CAPTURE.capture_scene()
                    STATE['shot'] = True
                    unreal.log('ROOM_CAPTURE_REQUESTED ' + str(PNG))
            elif elapsed > 27:
                if VIEWPORT_CAPTURE:
                    if not PNG.exists():
                        if elapsed > 240:
                            raise RuntimeError('Timed out waiting for native viewport screenshot')
                        return
                else:
                    unreal.RenderingLibrary.export_render_target(ELL.get_editor_world(), TARGET, str(OUT), PNG.name)
                if not PNG.exists() or PNG.stat().st_size < 1000:
                    raise RuntimeError('Render target export did not produce a PNG')
                unreal.log('ROOM_CAPTURE_DONE ' + str(PNG))
                if PNG.name.startswith('preview'):
                    STATE['idle'] = True
                    unreal.log('ROOM_CAPTURE_PREVIEW_READY; waiting for local capture command')
                else:
                    unreal.unregister_slate_post_tick_callback(HANDLE)
                    unreal.SystemLibrary.quit_editor()
            if elapsed > 240:
                raise RuntimeError('Timed out waiting for Unreal screenshot')
        except Exception:
            (OUT / 'capture_error.txt').write_text(traceback.format_exc())
            unreal.log_error(traceback.format_exc())
            unreal.unregister_slate_post_tick_callback(HANDLE)
            unreal.SystemLibrary.quit_editor()

    HANDLE = unreal.register_slate_post_tick_callback(tick)
except Exception:
    (OUT / 'capture_error.txt').write_text(traceback.format_exc())
    unreal.log_error(traceback.format_exc())
    unreal.SystemLibrary.quit_editor()
