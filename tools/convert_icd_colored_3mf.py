#!/usr/bin/env python3
"""Build compact colored OBJ visuals from the ICD SolidWorks 3MF export.

The source is an assembly in millimeters. ``--part-set`` can export the
vertically moving central body and the four wheel assemblies separately.
Geometry is only for rendering; Gazebo joints, inertia, and collision shapes
remain in the SDF model.
"""

import argparse
from collections import defaultdict
from pathlib import Path
import re
import shutil
import zipfile
import xml.etree.ElementTree as ET

import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy


SKIP_PART = re.compile(
    r"螺丝|螺母|垫片|轴承|销钉|螺栓|紧定|卡簧|弹垫|平垫|螺钉|"
    r"screw|nut|washer|bearing|bolt|skt|gbt|gb_\d|din\d|iso\d",
    re.IGNORECASE,
)
IDENTITY_3MF = "1 0 0 0 1 0 0 0 1 0 0 0"


def local_transform(text):
    values = np.fromstring(text or IDENTITY_3MF, sep=" ", dtype=np.float64)
    if len(values) != 12:
        raise ValueError(f"Invalid 3MF transform: {text}")
    # 3MF transforms operate on row vectors; translation is the last row.
    return values[:9].reshape(3, 3), values[9:]


def corners(bounds):
    low, high = bounds
    return np.array([[x, y, z] for x in (low[0], high[0])
                     for y in (low[1], high[1]) for z in (low[2], high[2])])


def reduce_mesh(vertices, faces, target):
    if len(faces) <= target:
        return vertices, faces
    compact, inverse = np.unique(faces.reshape(-1), return_inverse=True)
    vertices = vertices[compact]
    faces = inverse.reshape(-1, 3).astype(np.int64)
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(np.ascontiguousarray(vertices), deep=True))
    packed = np.column_stack((np.full(len(faces), 3), faces)).astype(np.int64)
    cells = vtk.vtkCellArray()
    cells.SetCells(len(faces), numpy_to_vtkIdTypeArray(packed.ravel(), deep=True))
    poly = vtk.vtkPolyData()
    poly.SetPoints(points)
    poly.SetPolys(cells)
    decimator = vtk.vtkQuadricDecimation()
    decimator.SetInputData(poly)
    decimator.SetTargetReduction(1.0 - target / len(faces))
    decimator.Update()
    result = decimator.GetOutput()
    if result.GetNumberOfPolys() == 0:
        return vertices, faces
    verts = vtk_to_numpy(result.GetPoints().GetData()).astype(np.float32, copy=True)
    tris = vtk_to_numpy(result.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    return verts, tris.astype(np.int32, copy=True)


def reduce_textured_mesh(vertices, faces, uv_faces, uv_table, target):
    # A CAD vertex may have different UVs across a texture seam. Give each
    # (vertex, UV) pair its own point before simplification.
    pairs, inverse = np.unique(
        np.column_stack((faces.reshape(-1), uv_faces.reshape(-1))),
        axis=0, return_inverse=True,
    )
    verts = vertices[pairs[:, 0]]
    coords = uv_table[pairs[:, 1]]
    tris = inverse.reshape(-1, 3).astype(np.int32)
    if len(tris) <= target:
        return verts, tris, coords
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(np.ascontiguousarray(verts), deep=True))
    packed = np.column_stack((np.full(len(tris), 3), tris)).astype(np.int64)
    cells = vtk.vtkCellArray()
    cells.SetCells(len(tris), numpy_to_vtkIdTypeArray(packed.ravel(), deep=True))
    poly = vtk.vtkPolyData()
    poly.SetPoints(points)
    poly.SetPolys(cells)
    tcoords = numpy_to_vtk(np.ascontiguousarray(coords, dtype=np.float32), deep=True)
    tcoords.SetName("TCoords")
    poly.GetPointData().SetTCoords(tcoords)
    decimator = vtk.vtkQuadricDecimation()
    decimator.SetInputData(poly)
    decimator.SetTargetReduction(1.0 - target / len(tris))
    decimator.AttributeErrorMetricOn()
    decimator.TCoordsAttributeOn()
    decimator.Update()
    result = decimator.GetOutput()
    if result.GetNumberOfPolys() == 0:
        return verts, tris, coords
    return (vtk_to_numpy(result.GetPoints().GetData()).astype(np.float32, copy=True),
            vtk_to_numpy(result.GetPolys().GetData()).reshape(-1, 4)[:, 1:].astype(np.int32, copy=True),
            vtk_to_numpy(result.GetPointData().GetTCoords()).astype(np.float32, copy=True))


def rgb_key(hex_color):
    value = (hex_color or "#999999").lstrip("#").upper()
    return "color_" + value[:6]


def color_value(resource, index):
    if resource is None or len(resource) == 0:
        return "#999999"
    index = min(max(int(index), 0), len(resource) - 1)
    entry = resource[index]
    return entry.get("displaycolor") or entry.get("color") or "#999999"


def load_groups(mesh_element, obj_element, resources):
    vertices = np.array([[float(v.get(k)) for k in ("x", "y", "z")]
                         for v in mesh_element.find("{*}vertices")], dtype=np.float32)
    raw = defaultdict(lambda: [[], []])
    default_pid = obj_element.get("pid")
    default_index = obj_element.get("pindex", "0")
    for tri in mesh_element.find("{*}triangles"):
        pid = tri.get("pid", default_pid)
        resource = resources.get(pid)
        typ = resource.tag.rsplit("}", 1)[-1] if resource is not None else ""
        face = (int(tri.get("v1")), int(tri.get("v2")), int(tri.get("v3")))
        if typ == "texture2dgroup":
            texture_id = resource.get("texid")
            key = ("texture_" + texture_id, pid)
            uv = tuple(int(tri.get(k, tri.get("p1", "0")))
                       for k in ("p1", "p2", "p3"))
        else:
            key = (rgb_key(color_value(resource, tri.get("p1", default_index))), None)
            uv = None
        raw[key][0].append(face)
        if uv is not None:
            raw[key][1].append(uv)
    groups = []
    for (material, uv_group), (faces, uv_faces) in raw.items():
        faces = np.asarray(faces, dtype=np.int32)
        if uv_group is None:
            target = max(80, min(2000, round(len(faces) * 0.15)))
            verts, faces = reduce_mesh(vertices, faces, target)
            groups.append((material, verts, faces, None))
        else:
            uv_table = np.array([[float(uv.get("u")), float(uv.get("v"))]
                                 for uv in resources[uv_group]], dtype=np.float32)
            target = max(80, min(2000, round(len(faces) * 0.15)))
            verts, faces, uv = reduce_textured_mesh(
                vertices, faces, np.asarray(uv_faces, dtype=np.int32), uv_table, target,
            )
            groups.append((material, verts, faces, uv))
    return groups


def write_vertex_lines(out, vertices):
    for x, y, z in vertices:
        out.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path, help="output .obj path")
    parser.add_argument("--chassis-height", type=float, default=0.076)
    parser.add_argument(
        "--part-set",
        choices=("all", "body", "wheels"),
        default="all",
        help=(
            "select the complete assembly, the vertically moving body, or the "
            "four top-level wheel assemblies"
        ),
    )
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(args.source) as archive:
        root = ET.fromstring(archive.read("3D/3dmodel.model"))
        if root.get("unit") != "millimeter":
            raise ValueError("Expected a millimeter 3MF assembly")
        resources_element = root.find("{*}resources")
        resources = {x.get("id"): x for x in resources_element if x.get("id")}
        objects = {}
        bounds = {}
        for obj in resources_element.findall("{*}object"):
            object_id = int(obj.get("id"))
            objects[object_id] = obj
            mesh = obj.find("{*}mesh")
            if mesh is not None:
                vertices = np.array([[float(v.get(k)) for k in ("x", "y", "z")]
                                     for v in mesh.find("{*}vertices")], dtype=np.float32)
                bounds[object_id] = (vertices.min(axis=0), vertices.max(axis=0))
        build = root.find("./{*}build/{*}item")
        root_id = int(build.get("objectid"))
        build_r, build_t = local_transform(build.get("transform"))
        instances = []
        low = np.full(3, np.inf)
        high = np.full(3, -np.inf)

        root_name = objects[root_id].get("name", "")
        wheel_path_prefix = f"/{root_name}/装配体4/"

        def visit(object_id, rotation, translation, path):
            nonlocal low, high
            obj = objects[object_id]
            name = obj.get("name", "")
            path = path + "/" + name
            mesh = obj.find("{*}mesh")
            if mesh is not None:
                extent = corners(bounds[object_id]) @ rotation + translation
                low = np.minimum(low, extent.min(axis=0))
                high = np.maximum(high, extent.max(axis=0))
                is_wheel_assembly = path.startswith(wheel_path_prefix)
                selected = (
                    args.part_set == "all"
                    or (args.part_set == "body" and not is_wheel_assembly)
                    or (args.part_set == "wheels" and is_wheel_assembly)
                )
                if selected and not SKIP_PART.search(path):
                    instances.append((object_id, rotation.copy(), translation.copy()))
                return
            for child in obj.find("{*}components"):
                child_r, child_t = local_transform(child.get("transform"))
                visit(int(child.get("objectid")), child_r @ rotation,
                      child_t @ rotation + translation, path)

        visit(root_id, build_r, build_t, "")
        center_x = (low[0] + high[0]) / 2
        ground_y = high[1]
        print(f"Assembly bounds (mm): {low} to {high}", flush=True)
        print(f"Rendering {len(instances)} non-fastener mesh instances", flush=True)

        materials = {}
        for resource_id, resource in resources.items():
            typ = resource.tag.rsplit("}", 1)[-1]
            if typ in ("colorgroup", "basematerials"):
                for entry in resource:
                    value = entry.get("displaycolor") or entry.get("color")
                    if value:
                        materials[rgb_key(value)] = value
            elif typ == "texture2dgroup":
                materials["texture_" + resource.get("texid")] = resource.get("texid")
        materials["color_999999"] = "#999999"

        obj_path = args.output
        mtl_path = obj_path.with_suffix(".mtl")
        for texture in resources_element.findall("{*}texture2d"):
            name = Path(texture.get("path")).name
            with archive.open(texture.get("path").lstrip("/")) as src:
                with (obj_path.parent / name).open("wb") as dst:
                    shutil.copyfileobj(src, dst)
        with mtl_path.open("w", encoding="utf-8") as mtl:
            for name, value in sorted(materials.items()):
                mtl.write(f"newmtl {name}\n")
                if name.startswith("texture_"):
                    mtl.write("Ka 1 1 1\nKd 1 1 1\n")
                    texture = resources[value]
                    mtl.write(f"map_Kd {Path(texture.get('path')).name}\n\n")
                else:
                    rgb = [int(value[i:i + 2], 16) / 255 for i in (1, 3, 5)]
                    mtl.write("Ka " + " ".join(f"{c:.5f}" for c in rgb) + "\n")
                    mtl.write("Kd " + " ".join(f"{c:.5f}" for c in rgb) + "\n\n")

        cached_groups = {}
        vertex_count = 0
        uv_count = 1
        normal_count = 0
        face_count = 0
        with obj_path.open("w", encoding="ascii") as out:
            out.write(f"mtllib {mtl_path.name}\n")
            # Even untextured submeshes need a UV set and normals: Ogre2
            # builds tangent buffers for every imported submesh.
            out.write("vt 0 0\n")
            for instance_index, (object_id, rotation, translation) in enumerate(instances, 1):
                if object_id not in cached_groups:
                    obj = objects[object_id]
                    cached_groups[object_id] = load_groups(obj.find("{*}mesh"), obj, resources)
                for group_index, (material, verts, faces, uv) in enumerate(cached_groups[object_id]):
                    transformed = verts @ rotation + translation
                    # CAD Y is downward; CAD -Z is the supplied front view.
                    gazebo = np.column_stack((
                        -transformed[:, 2], transformed[:, 0] - center_x,
                        ground_y - transformed[:, 1] - args.chassis_height * 1000,
                    )) * 0.001
                    # A distinct group keeps Ogre2 submeshes comfortably below
                    # its vertex-buffer limits, even for repeated CAD parts.
                    out.write(f"g part_{instance_index}_{group_index}\n")
                    write_vertex_lines(out, gazebo)
                    if uv is not None:
                        for u, v in uv:
                            out.write(f"vt {u:.6f} {v:.6f}\n")
                    out.write(f"usemtl {material}\n")
                    winding = (0, 1, 2) if np.linalg.det(rotation) > 0 else (0, 2, 1)
                    tri = gazebo[faces[:, winding]]
                    face_normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
                    norms = np.linalg.norm(face_normals, axis=1)
                    valid = norms > 1e-12
                    faces = faces[valid]
                    face_normals = face_normals[valid]
                    normals = np.zeros_like(gazebo)
                    for corner in range(3):
                        np.add.at(normals, faces[:, corner], face_normals)
                    lengths = np.linalg.norm(normals, axis=1)
                    used = lengths > 1e-12
                    normals[used] /= lengths[used, None]
                    normals[~used] = (0, 0, 1)
                    for nx, ny, nz in normals:
                        out.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")
                    for face in faces:
                        out.write("f " + " ".join(
                            f"{vertex_count + int(face[i]) + 1}/"
                            f"{uv_count + int(face[i]) + 1 if uv is not None else 1}/"
                            f"{normal_count + int(face[i]) + 1}" for i in winding
                        ) + "\n")
                    vertex_count += len(verts)
                    if uv is not None:
                        uv_count += len(uv)
                    normal_count += len(verts)
                    face_count += len(faces)
                if instance_index % 100 == 0:
                    print(f"{instance_index}/{len(instances)}: {face_count} triangles", flush=True)
        print(f"Wrote {obj_path}: {vertex_count} vertices, {face_count} triangles", flush=True)


if __name__ == "__main__":
    main()
