"""Render the original AirSim pawn with UE4's native transparent screenshot mask."""
import json
import math
import os
import time
import traceback
from pathlib import Path
import unreal

ROOT = Path('/home/ml/drone-stack-docker')
OUT = ROOT / 'paper_assets/airsim_drone'
ELL = unreal.EditorLevelLibrary
MAP = '/Game/DroneFigure/DroneOnly'
SOURCE = '/AirSim/Blueprints/BP_FlyingPawn'
SIZE = 4096


def spawn(cls, name, xyz, pitch=0, yaw=0):
    a = ELL.spawn_actor_from_class(cls, unreal.Vector(*xyz), unreal.Rotator(pitch=pitch, yaw=yaw, roll=0))
    a.set_actor_label(name)
    return a


try:
    OUT.mkdir(exist_ok=True, parents=True)
    ELL.new_level(MAP)
    DRONE = spawn(unreal.EditorAssetLibrary.load_blueprint_class(SOURCE), 'AirSim_OriginalDrone', (0, 0, 0))
    components = []
    for c in DRONE.get_components_by_class(unreal.StaticMeshComponent):
        c.set_render_custom_depth(True)
        c.set_forced_lod_model(1)
        components.append({'name': c.get_name(), 'mesh': str(c.get_editor_property('static_mesh')),
                           'location': str(c.get_world_location()), 'visible': c.is_visible()})
    for name, pitch, yaw, intensity in [('Key',-55,-35,5),('Fill',-40,140,3),('Rim',-35,55,2)]:
        a=spawn(unreal.DirectionalLight,name,(0,0,300),pitch,yaw)
        c=a.get_component_by_class(unreal.DirectionalLightComponent)
        c.set_mobility(unreal.ComponentMobility.MOVABLE)
        c.set_editor_property('intensity',float(intensity))
        c.set_editor_property('light_source_angle',8.0)
    sky=spawn(unreal.SkyLight,'Ambient',(0,0,350))
    sk=sky.get_component_by_class(unreal.SkyLightComponent)
    sk.set_mobility(unreal.ComponentMobility.MOVABLE)
    sk.set_editor_property('source_type',unreal.SkyLightSourceType.SLS_SPECIFIED_CUBEMAP)
    sk.set_editor_property('cubemap',unreal.load_asset('/Engine/MapTemplates/Sky/DaylightAmbientCubemap'))
    sk.set_intensity(1.0)
    sk.recapture_sky()
    centre, extent=DRONE.get_actor_bounds(False)
    distance=max(extent.x,extent.y,30)/math.tan(math.radians(20))*1.3+extent.z
    CAMERA=spawn(unreal.CameraActor,'DroneFigureCamera',(centre.x,centre.y,centre.z+distance),-90,0)
    cc=CAMERA.get_component_by_class(unreal.CameraComponent)
    cc.set_editor_property('field_of_view',40.0)
    cc.set_editor_property('aspect_ratio',1.0)
    cc.set_editor_property('constrain_aspect_ratio',True)
    post=cc.get_editor_property('post_process_settings')
    for name,value in [('auto_exposure_min_brightness',1.0),('auto_exposure_max_brightness',1.0),
                       ('auto_exposure_bias',0.0),('motion_blur_amount',0.0),('bloom_intensity',0.0)]:
        post.set_editor_property('override_'+name,True)
        post.set_editor_property(name,value)
    cc.set_editor_property('post_process_settings',post)
    cc.set_editor_property('post_process_blend_weight',1.0)
    ELL.set_selected_level_actors([])
    ELL.pilot_level_actor(CAMERA)
    ELL.editor_set_game_view(True)
    ELL.save_current_level()
    for command in ['r.Streaming.FullyLoadUsedTextures 1','r.Streaming.PoolSize 4096','r.MotionBlurQuality 0','r.BloomQuality 0','r.CustomDepth 3']:
        unreal.SystemLibrary.execute_console_command(ELL.get_editor_world(),command)
    (OUT/'capture_metadata.json').write_text(json.dumps({'source':SOURCE,'renderer':'UE4 native viewport',
        'resolution':[SIZE,SIZE],'background':'transparent, native custom-depth screenshot mask',
        'bounds':{'centre':str(centre),'extent':str(extent)},'camera_distance_cm':distance,
        'camera_pitch_deg':-90,'camera_fov_deg':40,'components':components},indent=2))
    START=time.monotonic()
    STATE={'stage':0,'task':None}
    PNG=OUT/'airsim_drone_top_transparent_4k.png'
    LIVE=ROOT/'.build/airsim-drone-capture/live_command.py'

    def tick(delta):
        try:
            elapsed=time.monotonic()-START
            if STATE['stage']==0 and elapsed>25:
                STATE['task']=unreal.AutomationLibrary.take_high_res_screenshot(SIZE,SIZE,str(PNG),camera=CAMERA,mask_enabled=True)
                STATE['stage']=1
            elif STATE['stage']==1 and PNG.exists():
                unreal.log('DRONE_CAPTURE_READY '+str(PNG))
                STATE['stage']=2
                if os.environ.get('DRONE_CAPTURE_HOLD', '0') != '1':
                    unreal.unregister_slate_post_tick_callback(HANDLE)
                    unreal.SystemLibrary.quit_editor()
            elif STATE['stage']==2 and LIVE.exists():
                command=LIVE.read_text()
                LIVE.rename(LIVE.with_suffix('.executed.py'))
                try:
                    exec(compile(command,str(LIVE),'exec'),globals())
                except Exception:
                    (OUT/'live_error.txt').write_text(traceback.format_exc())
            if elapsed>240 and STATE['stage']<2:
                raise RuntimeError('Screenshot timed out')
        except Exception:
            (OUT/'capture_error.txt').write_text(traceback.format_exc())
            unreal.unregister_slate_post_tick_callback(HANDLE)
            unreal.SystemLibrary.quit_editor()
    HANDLE=unreal.register_slate_post_tick_callback(tick)
except Exception:
    (OUT/'capture_error.txt').write_text(traceback.format_exc())
    unreal.SystemLibrary.quit_editor()
