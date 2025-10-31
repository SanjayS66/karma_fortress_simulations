#!/usr/bin/env python3
import subprocess, sys, xml.etree.ElementTree as ET, math, os

def render_xacro(path):
    try:
        p = subprocess.run(['xacro', path], capture_output=True, text=True, check=True)
        print("[info] xacro rendered successfully")
        return p.stdout
    except FileNotFoundError:
        print("[error] 'xacro' not found. Install ros-xacro or source your ROS environment.", file=sys.stderr); sys.exit(2)
    except subprocess.CalledProcessError as e:
        print("[error] xacro failed:", e.stderr.strip(), file=sys.stderr); sys.exit(3)

def find_wheel_joints(xmltext):
    root = ET.fromstring(xmltext)
    joints = []
    for j in root.findall('.//joint'):
        name = j.get('name','')
        origin = j.find('origin')
        if origin is None:
            continue
        xyz = origin.get('xyz','0 0 0').strip().split()
        # filter heuristics: name contains 'wheel' or 'Wheel' or 'Wheel_motor' or joint child named wheel*
        if ('wheel' in name.lower()) or ('motor' in name.lower()) or name.lower().startswith('wheel') or name.startswith('Wheel'):
            try:
                x,y,z = map(float, xyz)
            except:
                continue
            joints.append((name, x, y, z))
    return joints

def main(path):
    if not os.path.exists(path):
        print(f"[error] file not found: {path}", file=sys.stderr); sys.exit(1)
    xml = render_xacro(path)
    joints = find_wheel_joints(xml)
    print(f"[info] candidate wheel joints found: {len(joints)}")
    for n,x,y,z in joints:
        print(f"  {n}: x={x:.6f} y={y:.6f} z={z:.6f}")
    if not joints:
        print("[warn] no wheel-like joints found. Try relaxing heuristics or inspect URDF manually.")
        sys.exit(0)
    left = [j for j in joints if j[1] < 0]
    right = [j for j in joints if j[1] > 0]
    if not left or not right:
        print("[warn] could not partition wheels into left/right by x sign. Showing all joints above.")
    else:
        avg_left_x = sum(j[1] for j in left)/len(left)
        avg_right_x = sum(j[1] for j in right)/len(right)
        separation = abs(avg_right_x - avg_left_x)
        print(f"[result] avg_left_x={avg_left_x:.6f}, avg_right_x={avg_right_x:.6f}")
        print(f"[result] wheel_separation = {separation:.6f} m")
    # estimate radius as typical abs(z) of wheel origins
    radii = [abs(j[3]) for j in joints]
    est_r = sum(radii)/len(radii)
    print(f"[result] estimated_wheel_radius (avg abs z) = {est_r:.6f} m")
    print("[note] radius is approximate; verify with wheel geometry (visual/collision) if available")

if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv)>1 else "urdf/ArmPlate.urdf.xacro"
    main(path)