"""Generate /Game/Maps/BaselineMap using Unreal Editor's Python API.

Environment variables are used because UE4.27 command-line argument forwarding
is inconsistent between the editor and commandlet entry points.
"""

import os

import unreal


PAD_TEXTURE = os.environ["ARUCO_PAD_TEXTURE"]
PAD_SIZE_M = float(os.environ.get("ARUCO_PAD_SIZE_M", "0.8"))
FLOOR_SIZE_M = float(os.environ.get("ARUCO_FLOOR_SIZE_M", "8.0"))
FLOOR_THICKNESS_M = float(os.environ.get("ARUCO_FLOOR_THICKNESS_M", "0.1"))
FLOOR_GRAY = float(os.environ.get("ARUCO_FLOOR_GRAY", "0.45"))
PAD_HEIGHT_M = float(os.environ.get("ARUCO_PAD_HEIGHT_M", "0.002"))
COLLISION_ENABLED = os.environ.get("ARUCO_COLLISION_ENABLED", "1") == "1"

ASSET_PATH = "/Game/Baseline"
MAP_PATH = "/Game/Maps/BaselineMap"


def import_texture():
    task = unreal.AssetImportTask()
    task.set_editor_property("filename", PAD_TEXTURE)
    task.set_editor_property("destination_path", ASSET_PATH)
    task.set_editor_property("destination_name", "T_PaperPad")
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("save", True)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    texture = unreal.load_asset(ASSET_PATH + "/T_PaperPad")
    if texture is None:
        raise RuntimeError("failed to import pad texture: " + PAD_TEXTURE)
    texture.set_editor_property("mip_gen_settings", unreal.TextureMipGenSettings.TMGS_NO_MIPMAPS)
    # Bilinear sampling removes sub-pixel staircase shimmer during descent.
    # Mipmaps remain disabled so distant marker cells are never replaced by a
    # lower-resolution level; temporal filtering is controlled separately by
    # the landing camera settings.
    texture.set_editor_property("filter", unreal.TextureFilter.TF_BILINEAR)
    texture.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_DEFAULT)
    texture.set_editor_property("srgb", True)
    unreal.EditorAssetLibrary.save_loaded_asset(texture)
    return texture


def recreate_material(name):
    path = ASSET_PATH + "/" + name
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        unreal.EditorAssetLibrary.delete_asset(path)
    return unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        name, ASSET_PATH, unreal.Material, unreal.MaterialFactoryNew()
    )


def create_pad_material(texture):
    material = recreate_material("M_PaperPad_Unlit")
    material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
    material.set_editor_property("two_sided", True)
    sample = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionTextureSample, -250, 0
    )
    sample.set_editor_property("texture", texture)
    unreal.MaterialEditingLibrary.connect_material_property(
        sample, "RGB", unreal.MaterialProperty.MP_EMISSIVE_COLOR
    )
    unreal.MaterialEditingLibrary.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material)
    return material


def create_floor_material():
    material = recreate_material("M_FloorGray_Unlit")
    material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
    material.set_editor_property("two_sided", True)
    color = unreal.MaterialEditingLibrary.create_material_expression(
        material, unreal.MaterialExpressionConstant3Vector, -250, 0
    )
    color.set_editor_property(
        "constant", unreal.LinearColor(FLOOR_GRAY, FLOOR_GRAY, FLOOR_GRAY, 1.0)
    )
    unreal.MaterialEditingLibrary.connect_material_property(
        color, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR
    )
    unreal.MaterialEditingLibrary.recompile_material(material)
    unreal.EditorAssetLibrary.save_loaded_asset(material)
    return material


def spawn_mesh(label, mesh_path, material, location, scale):
    world = unreal.EditorLevelLibrary.get_editor_world()
    actor = unreal.BaselineEditorTools.spawn_actor_direct(
        world, unreal.StaticMeshActor, location, unreal.Rotator(0.0, 0.0, 0.0)
    )
    if actor is None:
        raise RuntimeError("failed to spawn " + label)
    actor.set_actor_label(label)
    actor.set_actor_scale3d(scale)
    component = actor.get_editor_property("static_mesh_component")
    component.set_static_mesh(unreal.load_asset(mesh_path))
    component.set_material(0, material)
    component.set_collision_enabled(
        unreal.CollisionEnabled.QUERY_AND_PHYSICS
        if COLLISION_ENABLED else unreal.CollisionEnabled.NO_COLLISION
    )
    actor.set_actor_enable_collision(COLLISION_ENABLED)
    if not COLLISION_ENABLED:
        component.set_collision_profile_name("NoCollision")
    return actor


def main():
    if not os.path.isfile(PAD_TEXTURE):
        raise RuntimeError("pad texture does not exist: " + PAD_TEXTURE)
    texture = import_texture()
    pad_material = create_pad_material(texture)
    floor_material = create_floor_material()

    if unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
        if not unreal.EditorLevelLibrary.load_level(MAP_PATH):
            raise RuntimeError("failed to load " + MAP_PATH)
        for actor in unreal.EditorLevelLibrary.get_all_level_actors():
            if actor.get_actor_label().startswith(
                ("BaselineFloor_", "PaperPad_L_", "AirSimNedOrigin_")
            ):
                unreal.BaselineEditorTools.destroy_actor_direct(actor)
    elif not unreal.EditorLevelLibrary.new_level(MAP_PATH):
        raise RuntimeError("failed to create " + MAP_PATH)

    # Engine basic shapes are one metre across. The floor cube top is exactly
    # z=0; the pad is raised 2 mm to avoid z-fighting while preserving scale.
    spawn_mesh(
        "BaselineFloor_%.2fm" % FLOOR_SIZE_M,
        "/Engine/BasicShapes/Cube.Cube",
        floor_material,
        unreal.Vector(0.0, 0.0, -FLOOR_THICKNESS_M * 50.0),
        unreal.Vector(FLOOR_SIZE_M, FLOOR_SIZE_M, FLOOR_THICKNESS_M),
    )
    spawn_mesh(
        "PaperPad_L_%.3fm" % PAD_SIZE_M,
        "/Engine/BasicShapes/Plane.Plane",
        pad_material,
        unreal.Vector(0.0, 0.0, PAD_HEIGHT_M * 100.0),
        unreal.Vector(PAD_SIZE_M, PAD_SIZE_M, 1.0),
    )

    world = unreal.EditorLevelLibrary.get_editor_world()
    start = unreal.BaselineEditorTools.spawn_actor_direct(
        world,
        unreal.PlayerStart,
        unreal.Vector(0.0, 0.0, 0.0),
        unreal.Rotator(0.0, 0.0, 0.0),
    )
    if start is None:
        raise RuntimeError("failed to spawn AirSim PlayerStart")
    start.set_actor_label("AirSimNedOrigin_PadCenter")

    game_mode = unreal.load_class(None, "/Script/AirSim.AirSimGameMode")
    if game_mode is None:
        raise RuntimeError("AirSimGameMode is unavailable; check the project plugin link")
    world.get_world_settings().set_editor_property("default_game_mode", game_mode)
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError("failed to save " + MAP_PATH)
    unreal.log(
        "Generated %s: pad L=%.3fm, floor=%.3fm, collision=%s, origin=(0,0,0)"
        % (MAP_PATH, PAD_SIZE_M, FLOOR_SIZE_M, COLLISION_ENABLED)
    )


main()
