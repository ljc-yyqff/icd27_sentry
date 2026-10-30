#!/usr/bin/env python3
"""Convert the ICD assembly's Bambu 3MF into a Gazebo sized visual mesh.

The 3MF contains ~40 million triangles. Process one component at a time so the
conversion does not need to unpack or hold the complete assembly in memory.
"""

import argparse
from array import array
import struct
from pathlib import Path
from xml.parsers import expat
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy


SKIP_PART_NAMES = (
    "螺丝", "螺母", "垫片", "轴承", "销钉", "螺栓", "紧定", "卡簧",
    "弹垫", "平垫", "螺钉",
)


def reduce_mesh(vertices, triangles, target_faces):
    if len(triangles) <= target_faces:
        return vertices, triangles
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(vertices, deep=True))
    cells = vtk.vtkCellArray()
    packed = np.empty((len(triangles), 4), dtype=np.int64)
    packed[:, 0] = 3
    packed[:, 1:] = triangles
    cells.SetCells(len(triangles), numpy_to_vtkIdTypeArray(packed.ravel(), deep=True))
    poly = vtk.vtkPolyData()
    poly.SetPoints(points)
    poly.SetPolys(cells)
    decimator = vtk.vtkQuadricDecimation()
    decimator.SetInputData(poly)
    decimator.SetTargetReduction(1.0 - target_faces / len(triangles))
    decimator.Update()
    result = decimator.GetOutput()
    verts = vtk_to_numpy(result.GetPoints().GetData()).astype(np.float32, copy=True)
    faces = vtk_to_numpy(result.GetPolys().GetData()).reshape(-1, 4)[:, 1:].astype(np.int32, copy=True)
    return verts, faces


def write_stl_faces(out, vertices, faces, offset):
    if len(faces) == 0:
        return 0
    # The CAD file uses Y downward; the supplied top/bottom reference views
    # confirm this. Gazebo uses Z upward. Rotate the other two axes so that
    # negative CAD Z faces forward and the transform remains right handed.
    # The mesh is attached to chassis, 76 mm above base_footprint.
    verts = np.empty_like(vertices)
    verts[:, 0] = -vertices[:, 2] - offset[2]
    verts[:, 1] = vertices[:, 0] + offset[0]
    verts[:, 2] = -vertices[:, 1] - offset[1] + 125.349225 - 76.0
    verts *= 0.001
    tri = verts[faces]
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(lengths[:, None], 1e-20)
    payload = np.zeros(len(faces), dtype=np.dtype([('normal', '<f4', (3,)),
                                                  ('vertices', '<f4', (3, 3)),
                                                  ('attribute', '<u2')]))
    payload['normal'] = normals
    payload['vertices'] = tri
    out.write(payload.tobytes())
    return len(faces)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('target', type=Path)
    parser.add_argument('--max-parts', type=int, default=0, help='for a quick test')
    args = parser.parse_args()
    args.target.parent.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(args.source) as archive:
        assembly = ET.fromstring(archive.read('3D/3dmodel.model'))
        settings = ET.fromstring(archive.read('Metadata/model_settings.config'))
        namespace = {'m': 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
        offsets = {}
        for component in assembly.findall('./m:resources/m:object/m:components/m:component', namespace):
            values = [float(value) for value in component.get('transform').split()]
            if not np.allclose(values[:9], [1, 0, 0, 0, 1, 0, 0, 0, 1]):
                raise ValueError('Unexpected rotated component in 3MF')
            offsets[int(component.get('objectid'))] = values[9:12]

        selected = {}
        for part in settings.find('object').findall('part'):
            metadata = {entry.get('key'): entry.get('value') for entry in part.findall('metadata')}
            name = metadata.get('name', '')
            if not any(word in name for word in SKIP_PART_NAMES):
                selected[int(part.get('id'))] = (name, int(part.find('mesh_stat').get('face_count')))
        print(f'Selected {len(selected)} components from {len(offsets)}', flush=True)

        with args.target.open('wb') as out:
            out.write(b'ICD sentry assembly 3MF visual'.ljust(80, b'\0'))
            out.write(struct.pack('<I', 0))
            total_faces = 0
            processed = 0
            current_id = None
            xyz = array('f')
            indices = array('I')

            def start(tag, attrs):
                nonlocal current_id, xyz, indices
                if tag == 'object':
                    current_id = int(attrs['id'])
                    xyz = array('f')
                    indices = array('I')
                elif current_id in selected:
                    if tag == 'vertex':
                        xyz.extend((float(attrs['x']), float(attrs['y']), float(attrs['z'])))
                    elif tag == 'triangle':
                        indices.extend((int(attrs['v1']), int(attrs['v2']), int(attrs['v3'])))

            def end(tag):
                nonlocal current_id, xyz, indices, total_faces, processed
                if tag != 'object':
                    return
                if current_id in selected:
                    vertices = np.frombuffer(xyz, dtype=np.float32).reshape(-1, 3)
                    faces = np.frombuffer(indices, dtype=np.uint32).reshape(-1, 3)
                    if len(faces):
                        # Preserve at least a little geometry from each part.
                        budget = max(80, min(5000, round(len(faces) * 0.05)))
                        simple_vertices, simple_faces = reduce_mesh(vertices, faces, budget)
                        total_faces += write_stl_faces(out, simple_vertices, simple_faces,
                                                       offsets[current_id])
                    processed += 1
                    if processed % 100 == 0:
                        print(f'{processed}/{len(selected)} parts, {total_faces} faces', flush=True)
                current_id = None
                xyz = array('f')
                indices = array('I')
                if args.max_parts and processed >= args.max_parts:
                    raise StopIteration

            xml_parser = expat.ParserCreate()
            xml_parser.StartElementHandler = start
            xml_parser.EndElementHandler = end
            try:
                with archive.open('3D/Objects/object_1.model') as stream:
                    while chunk := stream.read(4 * 1024 * 1024):
                        xml_parser.Parse(chunk, False)
                    xml_parser.Parse(b'', True)
            except StopIteration:
                pass
            out.seek(80)
            out.write(struct.pack('<I', total_faces))
    print(f'Wrote {args.target}: {processed} parts, {total_faces} faces', flush=True)


if __name__ == '__main__':
    main()
