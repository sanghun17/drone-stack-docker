"""Render the same metric marker model used by PhysicalPadDetector.

No pose/corner ground truth is passed to the detector. Black/white cells are USD
geometry, so the renderer supplies the optical observation through the camera.
"""
import cv2
import numpy as np

MARKER_PLANE_Z_M = .002
PAPER_PLANE_Z_M = .001


def metric_pad_manifest(manifest, pad_side_m=None):
    """Convert nominal print coordinates to the existing camera/pad convention.

    Print image right is pad -Y, and image up is pad +X. Existing calibrated
    runtime manifests already use this convention and pass through unchanged.
    """
    if 'canvas_units' not in manifest:
        return manifest
    if pad_side_m is None or pad_side_m <= 0:
        raise ValueError('nominal print layout requires a positive pad side')
    canvas = float(manifest['canvas_units'])
    scale = pad_side_m / canvas
    markers = []
    for marker in manifest['markers']:
        size = float(marker['size'])
        markers.append(dict(id=int(marker['id']), side_m=size*scale,
            center_m=dict(x=(canvas/2-marker['y']-size/2)*scale,
                          y=(canvas/2-marker['x']-size/2)*scale), yaw_deg=-90.))
    return dict(name=manifest['name'], dictionary=manifest['dictionary'],
                pad_side_m=pad_side_m, markers=markers)


def marker_cells(manifest):
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, manifest['dictionary']))
    quads = []
    for marker in manifest['markers']:
        bits = cv2.aruco.generateImageMarker(dictionary, marker['id'], 6)
        a = np.radians(marker['yaw_deg'])
        R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        center = np.array([marker['center_m']['x'], marker['center_m']['y']])
        for row, col in np.argwhere(bits == 0):
            x, y = col/6-.5, .5-row/6
            # Counterclockwise vertices with outward +Z face normal.
            square = np.array([[x,y-1/6],[x+1/6,y-1/6],[x+1/6,y],[x,y]])
            quads.append(square @ R.T * marker['side_m'] + center)
    return np.asarray(quads)


def add_pad(stage, env_count, manifest):
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    quads = marker_cells(manifest)
    extent = max(.5, float(np.abs(quads).max())+.025)
    root = '/World/envs/env_0/Pad'
    UsdGeom.Xform.Define(stage, root)
    for label, color in [('White', 1.0), ('Black', 0.0)]:
        material = UsdShade.Material.Define(stage, root+'/'+label+'Material')
        shader = UsdShade.Shader.Define(stage, root+'/'+label+'Material/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(color))
        shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(color))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(1.)
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        mesh = UsdGeom.Mesh.Define(stage, root+'/'+label)
        cells = quads if label == 'Black' else np.array([[[-extent,-extent],[extent,-extent],[extent,extent],[-extent,extent]]])
        z = MARKER_PLANE_Z_M if label == 'Black' else PAPER_PLANE_Z_M
        vertices = np.c_[cells.reshape(-1,2), np.full(len(cells)*4,z)]
        mesh.CreatePointsAttr(vertices.tolist())
        mesh.CreateFaceVertexCountsAttr([4]*len(cells))
        mesh.CreateFaceVertexIndicesAttr(list(range(len(cells)*4)))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(False)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    # Internal references share the visual asset; each env still owns its pose
    # and scene-partition membership.
    for env in range(1, env_count):
        stage.DefinePrim('/World/envs/env_%d/Pad' % env, 'Xform').GetReferences().AddInternalReference(root)
