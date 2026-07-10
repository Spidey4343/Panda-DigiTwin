#!/usr/bin/env python3
"""
Convert Franka FR3 DAE meshes to VRML97 WRL format.
Run from inside panda-digitwin/:  python3 convert_meshes.py
"""
import os, sys, xml.etree.ElementTree as ET

def parse_floats(text):
    return [float(x) for x in text.strip().split()]

def parse_ints(text):
    return [int(x) for x in text.strip().split()]

def dae_to_wrl(dae_path, wrl_path):
    print(f"  {os.path.basename(dae_path)} → {os.path.basename(wrl_path)}", end=" ", flush=True)
    try:
        tree = ET.parse(dae_path)
        root = tree.getroot()
    except Exception as e:
        print(f"\n  ERROR: {e}")
        return False

    # Detect namespace
    ns = root.tag.split('}')[0] + '}' if root.tag.startswith('{') else ''
    T  = lambda tag: f'{ns}{tag}'

    all_shapes = []

    for geom in root.findall(f'.//{T("geometry")}'):
        mesh = geom.find(T('mesh'))
        if mesh is None:
            continue

        # Collect all sources
        sources = {}
        for src in mesh.findall(T('source')):
            sid = src.get('id','')
            fa  = src.find(T('float_array'))
            if fa is not None and fa.text:
                sources[sid] = parse_floats(fa.text)

        # Find vertices source (positions)
        verts_el = mesh.find(T('vertices'))
        pos_src_id = None
        if verts_el is not None:
            for inp in verts_el.findall(T('input')):
                if inp.get('semantic') == 'POSITION':
                    pos_src_id = inp.get('source','').lstrip('#')

        if not pos_src_id or pos_src_id not in sources:
            # Try direct position source
            for k in sources:
                if 'position' in k.lower():
                    pos_src_id = k
                    break

        if not pos_src_id:
            continue

        positions = sources[pos_src_id]
        verts_id  = verts_el.get('id','') if verts_el is not None else ''

        # Process triangles and polylist
        for prim_tag in ['triangles', 'polylist', 'trifans', 'tristrips']:
            for prim in mesh.findall(T(prim_tag)):
                inputs   = prim.findall(T('input'))
                stride   = max(int(i.get('offset',0)) for i in inputs) + 1
                
                vert_off = 0
                for inp in inputs:
                    sem = inp.get('semantic','')
                    src = inp.get('source','').lstrip('#')
                    if sem == 'VERTEX':
                        vert_off = int(inp.get('offset', 0))

                p_el = prim.find(T('p'))
                if p_el is None or not p_el.text:
                    continue
                raw = parse_ints(p_el.text)

                face_indices = []

                if prim_tag == 'triangles':
                    for i in range(0, len(raw), stride * 3):
                        try:
                            i0 = raw[i + vert_off]
                            i1 = raw[i + stride + vert_off]
                            i2 = raw[i + stride*2 + vert_off]
                            face_indices.extend([i0, i1, i2, -1])
                        except IndexError:
                            break

                elif prim_tag == 'polylist':
                    vc_el = prim.find(T('vcount'))
                    if vc_el is None or not vc_el.text:
                        continue
                    vcounts = parse_ints(vc_el.text)
                    idx = 0
                    for vc in vcounts:
                        face = []
                        for j in range(vc):
                            face.append(raw[idx + vert_off])
                            idx += stride
                        for k in range(1, len(face)-1):
                            face_indices.extend([face[0], face[k], face[k+1], -1])

                if face_indices and positions:
                    all_shapes.append({
                        'positions': positions,
                        'indices':   face_indices,
                    })

    if not all_shapes:
        print("WARNING: no geometry found")
        return False

    with open(wrl_path, 'w') as f:
        f.write('#VRML V2.0 utf8\n\n')
        f.write('Group {\n  children [\n')

        for shape in all_shapes:
            pos = shape['positions']
            idx = shape['indices']

            f.write('    Shape {\n')
            f.write('      appearance Appearance {\n')
            f.write('        material Material {\n')
            f.write('          diffuseColor  0.72 0.72 0.76\n')
            f.write('          specularColor 0.4  0.4  0.45\n')
            f.write('          shininess     0.4\n')
            f.write('          ambientIntensity 0.35\n')
            f.write('        }\n')
            f.write('      }\n')
            f.write('      geometry IndexedFaceSet {\n')
            f.write('        solid FALSE\n')
            f.write('        creaseAngle 0.785\n')
            f.write('        coord Coordinate {\n')
            f.write('          point [\n')

            for i in range(0, len(pos)-2, 3):
                f.write(f'            {pos[i]:.5f} {pos[i+1]:.5f} {pos[i+2]:.5f}')
                f.write(',\n' if i + 3 < len(pos) else '\n')

            f.write('          ]\n        }\n')
            f.write('        coordIndex [\n')

            line = []
            for v in idx:
                line.append(str(v))
                if v == -1:
                    f.write('          ' + ' '.join(line) + '\n')
                    line = []
            if line:
                f.write('          ' + ' '.join(line) + '\n')

            f.write('        ]\n      }\n    }\n')

        f.write('  ]\n}\n')

    kb = os.path.getsize(wrl_path) // 1024
    print(f"✓ {kb} KB")
    return True


def main():
    base = os.path.dirname(os.path.abspath(__file__))
    meshes = os.path.join(base, 'meshes')

    if not os.path.exists(meshes):
        print(f"ERROR: meshes/ not found at {meshes}")
        sys.exit(1)

    files = ['link0','link1','link2','link3','link4','link5','link6','link7','hand','finger']
    print("=== DAE → WRL Converter ===\n")
    ok, fail = 0, []

    for name in files:
        dae = os.path.join(meshes, f'{name}.dae')
        wrl = os.path.join(meshes, f'{name}.wrl')
        if not os.path.exists(dae):
            print(f"  SKIP: {name}.dae not found")
            fail.append(name)
            continue
        if dae_to_wrl(dae, wrl):
            ok += 1
        else:
            fail.append(name)

    print(f"\n=== {ok} converted, {len(fail)} skipped/failed ===")
    if fail:
        print(f"  Skipped: {', '.join(fail)}")
    print("\nNow run:  python3 -m http.server 8080")
    print("Then open: http://localhost:8080")

if __name__ == '__main__':
    main()
