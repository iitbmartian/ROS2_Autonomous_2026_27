#!/usr/bin/env python3
"""Generate the terrain test worlds for 3D navigation: slopes, crater_field, rock_field, urc_mix.

    python3 make_terrain_worlds.py            # writes worlds/*.sdf and models/Terrain*/

Run it again after changing any constant below. SEED fixes every random layout, so two people
running this get the same worlds.

Smooth terrain is a triangle mesh (binary STL) built from a height field, so ramps, sidehills
and craters are continuous surfaces a wheel can roll over, not stacks of boxes with step edges.
The same mesh is the collision and the visual geometry. Gazebo heightmaps were tried first and
are not accurate enough here: on slopes the rover rested 5.3 cm above the heightmap's surface
(DART collides heightmaps through Bullet, with its collision margin) while the lidar saw the
rendered ground 2.5 cm below it. Rocks are boxes and ellipsoids, as in marsyard.sdf. Each mesh
is a model under models/ (on GZ_SIM_RESOURCE_PATH through this package's environment hook),
referenced as model://<name>/meshes/terrain.stl.

The worlds feed the 3D navigation benchmark (rover_nav2/doc/3d_nav/SPEC.md). Its ground-truth
height export (rover_nav2 benchmark ground_truth.py) reads these same SDF files, heightmaps and
primitives, so the benchmark scores against exactly the terrain Gazebo simulates.

Heightmap layout in Gazebo: an N x N image (N = 2^k + 1) spans <size> x <size> metres centred on
<pos>. Row 0 of the image is the +y edge and column 0 the -x edge. Pixel value v in [0, 65535]
maps to height pos.z + v / 65535 * size.z.
"""

from __future__ import annotations

import math
import os
import random

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SEED = 20261101

RES = 0.05                     # m per heightmap pixel, the 3D stack's map resolution
REGOLITH = 'model://MarsyardGround/materials/textures/regolith.png'
CLEAR_RADIUS = 2.5             # m kept flat and rock-free round the spawn point


# ---------------------------------------------------------------------------------------------
# Height fields
# ---------------------------------------------------------------------------------------------

class Field:
    """A square height field of n x n samples, spacing RES, centred on (cx, cy)."""

    def __init__(self, n: int, cx: float, cy: float):
        self.n, self.cx, self.cy = n, cx, cy
        self.size = (n - 1) * RES
        half = self.size / 2.0
        xs = cx - half + np.arange(n) * RES
        ys = cy + half - np.arange(n) * RES          # row 0 is the +y edge
        self.x, self.y = np.meshgrid(xs, ys)
        self.z = np.zeros((n, n))


def smoothstep(e0, e1, v):
    t = np.clip((v - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def soften(z: np.ndarray, radius_px: int) -> np.ndarray:
    """Box blur, so creases where a ramp meets the ground are wheel-scale curves, not kinks."""
    if radius_px <= 0:
        return z
    k = 2 * radius_px + 1
    pad = np.pad(z, radius_px, mode='edge')
    c = np.cumsum(np.cumsum(pad, axis=0), axis=1)
    c = np.pad(c, ((1, 0), (1, 0)))
    return (c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]) / (k * k)


def add_ramp_lane(f: Field, y0: float, width: float, x0: float, angle_deg: float,
                  height: float, plateau: float):
    """Ramp up at angle_deg from x0 to a plateau of the given height and length, then down."""
    run = height / math.tan(math.radians(angle_deg))
    x1, x2, x3 = x0 + run, x0 + run + plateau, x0 + 2 * run + plateau
    prof = np.where(f.x < x0, 0.0,
            np.where(f.x < x1, (f.x - x0) / run * height,
             np.where(f.x < x2, height,
              np.where(f.x < x3, (x3 - f.x) / run * height, 0.0))))
    # Lane sides fall off over 0.3 m, so the lane is a raised causeway the rover must stay on.
    across = 1.0 - smoothstep(width / 2.0, width / 2.0 + 0.3, np.abs(f.y - y0))
    f.z = np.maximum(f.z, prof * across)


def add_sidehill(f: Field, x0: float, x1: float, y_low: float, y_high: float, angle_deg: float):
    """A slope rising in +y at angle_deg from y_low to y_high across x0..x1, holding its top
    height as a plateau beyond y_high. Its x ends taper so their own slope stays under 15 deg:
    a smoothstep over L peaks at 1.5 h / L, so L = 1.5 h / tan(15 deg)."""
    h = (y_high - y_low) * math.tan(math.radians(angle_deg))
    rise = (np.clip(f.y, y_low, y_high) - y_low) * math.tan(math.radians(angle_deg))
    taper = 1.5 * h / math.tan(math.radians(15.0))
    along = smoothstep(x0 - taper, x0, f.x) * (1.0 - smoothstep(x1, x1 + taper, f.x))
    f.z = np.maximum(f.z, rise * along)


def crater(f: Field, x: float, y: float, diameter: float, depth: float, rim: float):
    """Bowl of the given depth below the surroundings, with a raised rim falling off outside."""
    R = diameter / 2.0
    r = np.hypot(f.x - x, f.y - y)
    inside = rim - (rim + depth) * (1.0 - (r / R) ** 2)
    outside = rim * np.exp(-((r - R) / (0.35 * R)) ** 2)
    f.z += np.where(r < R, inside, outside)


def hills(f: Field, rng: np.random.Generator, amplitude: float, wavelength: float):
    """Smooth random undulation: a few sinusoids of about the given wavelength."""
    z = np.zeros_like(f.z)
    for _ in range(6):
        k = 2 * math.pi / (wavelength * rng.uniform(0.7, 1.5))
        th = rng.uniform(0, 2 * math.pi)
        ph = rng.uniform(0, 2 * math.pi)
        z += np.sin(k * (f.x * math.cos(th) + f.y * math.sin(th)) + ph)
    f.z += amplitude * z / 6.0 * 2.0


def flatten_spawn(f: Field, radius: float = CLEAR_RADIUS):
    r = np.hypot(f.x, f.y)
    w = smoothstep(radius, radius + 1.5, r)
    f.z *= w


# ---------------------------------------------------------------------------------------------
# Rocks
# ---------------------------------------------------------------------------------------------

def rock_sdf(i: int, x: float, y: float, base_z: float, rng: random.Random,
             size_min: float, size_max: float, h_min: float, h_max: float) -> str:
    sx = rng.uniform(size_min, size_max)
    sy = sx * rng.uniform(0.6, 1.4)
    sz = sx * rng.uniform(h_min, h_max)
    yaw = rng.uniform(0, 2 * math.pi)
    shade = rng.uniform(0.28, 0.72)
    r, g, b = min(shade * rng.uniform(0.85, 1.15), 1.0), shade * 0.82, shade * 0.70
    if rng.random() < 0.45:
        geom = f'<ellipsoid><radii>{sx / 2:.3f} {sy / 2:.3f} {sz / 2:.3f}</radii></ellipsoid>'
        # Half buried, like a real boulder: the visible height is sz / 2.
        z = base_z
    else:
        geom = f'<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>'
        z = base_z + sz / 2.0 - 0.02
    return f"""    <model name="rock_{i:03d}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def scatter_rocks(rng: random.Random, count: int, x_range, y_range, height_at, **sizes):
    out = []
    for i in range(count):
        while True:
            x, y = rng.uniform(*x_range), rng.uniform(*y_range)
            if math.hypot(x, y) > CLEAR_RADIUS + 0.5:
                break
        out.append(rock_sdf(i, x, y, height_at(x, y), rng, **sizes))
    return out


# ---------------------------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------------------------

def write_heightmap_model(name: str, f: Field, description: str) -> tuple[str, float, float]:
    """Write models/<name>/ with a 16-bit heightmap. Returns (uri, z_min, z_range)."""
    d = os.path.join(PKG, 'models', name)
    os.makedirs(d, exist_ok=True)
    z_min, z_max = float(f.z.min()), float(f.z.max())
    z_range = max(z_max - z_min, 1e-3)
    img = np.round((f.z - z_min) / z_range * 65535.0).astype(np.uint16)
    Image.fromarray(img, mode='I;16').save(os.path.join(d, 'heightmap.png'))
    with open(os.path.join(d, 'model.config'), 'w') as fh:
        fh.write(f"""<?xml version="1.0"?>
<!-- GENERATED by tools/make_terrain_worlds.py. -->
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>{description} Heightmap only, used by the worlds of the same name.</description>
</model>
""")
    with open(os.path.join(d, 'model.sdf'), 'w') as fh:
        fh.write(f"""<?xml version="1.0"?>
<!-- GENERATED by tools/make_terrain_worlds.py. The heightmap is used directly by the world
     file, this model only makes the directory a valid model:// path. -->
<sdf version="1.8"><model name="{name}"><static>true</static><link name="link"/></model></sdf>
""")
    return f'model://{name}/heightmap.png', z_min, z_range


def write_mesh_model(name: str, f: Field, description: str, stride: int) -> str:
    """Write models/<name>/ with the height field as a binary STL, sampled every stride cells.
    Returns the mesh uri."""
    d = os.path.join(PKG, 'models', name)
    os.makedirs(os.path.join(d, 'meshes'), exist_ok=True)
    for stale in ('heightmap.png',):
        if os.path.exists(os.path.join(d, stale)):
            os.remove(os.path.join(d, stale))
    x, y, z = f.x[::stride, ::stride], f.y[::stride, ::stride], f.z[::stride, ::stride]
    v = np.stack([x, y, z], axis=-1)
    a, b = v[:-1, :-1], v[:-1, 1:]
    c, e = v[1:, :-1], v[1:, 1:]
    # Rows run toward -y and columns toward +x, so (a, c, b) and (b, c, e) wind counter-clockwise
    # seen from above: normals point up.
    tris = np.concatenate([np.stack([a, c, b], axis=-2).reshape(-1, 3, 3),
                           np.stack([b, c, e], axis=-2).reshape(-1, 3, 3)])
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True)
    rec = np.zeros(len(tris), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'], rec['v'] = n, tris
    with open(os.path.join(d, 'meshes', 'terrain.stl'), 'wb') as fh:
        fh.write(b'make_terrain_worlds.py terrain'.ljust(80, b' '))
        fh.write(np.uint32(len(tris)).tobytes())
        fh.write(rec.tobytes())
    with open(os.path.join(d, 'model.config'), 'w') as fh:
        fh.write(f"""<?xml version="1.0"?>
<!-- GENERATED by tools/make_terrain_worlds.py. -->
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>{description} Terrain mesh, used by the world of the same name.</description>
</model>
""")
    with open(os.path.join(d, 'model.sdf'), 'w') as fh:
        fh.write(f"""<?xml version="1.0"?>
<!-- GENERATED by tools/make_terrain_worlds.py. The mesh is used directly by the world file, this
     model only makes the directory a valid model:// path. -->
<sdf version="1.8"><model name="{name}"><static>true</static><link name="link"/></model></sdf>
""")
    print(f'wrote models/{name}/meshes/terrain.stl: {len(tris)} triangles at {RES * stride:.2f} m')
    return f'model://{name}/meshes/terrain.stl'


def mesh_terrain_model(uri: str) -> str:
    geom = f'<mesh><uri>{uri}</uri></mesh>'
    return f"""    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>{geom}</geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material><ambient>0.50 0.40 0.32 1</ambient><diffuse>0.62 0.50 0.40 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def terrain_model(uri: str, f: Field, z_min: float, z_range: float) -> str:
    geom = f"""<heightmap>
            <uri>{uri}</uri>
            <size>{f.size:.3f} {f.size:.3f} {z_range:.4f}</size>
            <pos>{f.cx:.3f} {f.cy:.3f} {z_min:.4f}</pos>"""
    return f"""    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
          {geom}
          </heightmap>
          </geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry>
          {geom}
            <texture>
              <diffuse>{REGOLITH}</diffuse>
              <normal>model://TerrainFlatNormal/flat_normal.png</normal>
              <size>4</size>
            </texture>
          </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
"""


def ground_plane(z: float) -> str:
    """Infinite plane under the terrain, so a rover that leaves the heightmap does not fall
    forever. Below the lowest terrain point, so it never props up a crater floor."""
    return f"""    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
          <material><ambient>0.45 0.38 0.32 1</ambient><diffuse>0.55 0.45 0.38 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def world(name: str, comment: str, models: list[str]) -> str:
    body = ''.join(models)
    return f"""<?xml version="1.0"?>
<!--
  GENERATED by tools/make_terrain_worlds.py. Edit that, not this.

{comment}

  Declares its systems explicitly for the same reason the other worlds do: the Sensors and Imu
  systems the rover needs are not in Gazebo's default set, and naming any plugin disables the
  default set, so the core trio goes in too.
-->
<sdf version="1.8">
  <world name="{name}">

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -9.80665</gravity>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>1.0 0.94 0.86 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.6 0.35 -0.72</direction>
    </light>

    <scene>
      <ambient>0.45 0.42 0.40 1</ambient>
      <background>0.55 0.42 0.34 1</background>
    </scene>

{body}  </world>
</sdf>
"""


def write_world(name: str, text: str):
    with open(os.path.join(PKG, 'worlds', f'{name}.sdf'), 'w') as fh:
        fh.write(text)
    print(f'wrote worlds/{name}.sdf')


def flat_normal_model():
    d = os.path.join(PKG, 'models', 'TerrainFlatNormal')
    os.makedirs(d, exist_ok=True)
    img = np.zeros((64, 64, 3), dtype=np.uint8)
    img[:] = (128, 128, 255)
    Image.fromarray(img, mode='RGB').save(os.path.join(d, 'flat_normal.png'))
    with open(os.path.join(d, 'model.config'), 'w') as fh:
        fh.write("""<?xml version="1.0"?>
<!-- GENERATED by tools/make_terrain_worlds.py. -->
<model><name>TerrainFlatNormal</name><version>1.0</version><sdf version="1.8">model.sdf</sdf>
  <description>Flat normal map for heightmap textures, which require one.</description></model>
""")
    with open(os.path.join(d, 'model.sdf'), 'w') as fh:
        fh.write('<?xml version="1.0"?>\n<sdf version="1.8"><model name="TerrainFlatNormal">'
                 '<static>true</static><link name="link"/></model></sdf>\n')


# ---------------------------------------------------------------------------------------------
# The worlds
# ---------------------------------------------------------------------------------------------

def make_slopes():
    f = Field(513, 9.0, 0.0)                    # 25.6 m square, x -3.8..21.8, y -12.8..12.8
    lanes = [(0.0, 15.0), (-5.0, 25.0), (-10.0, 35.0)]
    for y0, ang in lanes:
        add_ramp_lane(f, y0, width=2.5, x0=4.0, angle_deg=ang, height=0.8, plateau=3.0)
    add_sidehill(f, x0=7.0, x1=14.0, y_low=6.0, y_high=9.0, angle_deg=20.0)
    f.z = soften(f.z, 2)
    flatten_spawn(f)
    uri = write_mesh_model(
        'TerrainSlopes', f, 'Ramps at 15, 25 and 35 degrees and a 20 degree sidehill.', 2)
    z0 = float(f.z.min())
    comment = """  Slopes for terrain-aware navigation. Spawn at the origin, all lanes run along +x.

    y =   0     15 deg ramp up to a 0.8 m plateau (x 4 to 10), and down the far side
    y =  -5     25 deg, same plateau
    y = -10     35 deg, same plateau (steeper than the rover should ever climb)
    y = 6..9    20 degree sidehill rising in +y, full slope for x 7 to 14, tapering in over
                about 6 m at each end, with a plateau beyond y 9.
                Driven along x it rolls the rover 20 degrees, beyond the roll limit, while
                facing uphill it pitches 20 degrees, within the pitch limit.

  Lanes are raised causeways 2.5 m wide with sides falling off over 0.3 m."""
    write_world('slopes', world('slopes', comment,
                                [ground_plane(z0 - 0.5), mesh_terrain_model(uri)]))


def make_crater_field():
    rng = random.Random(SEED + 1)
    f = Field(513, 9.0, 0.0)
    craters = []
    # Sizes from smaller than a wheel to several times the track (0.436 m).
    sizes = [0.3] * 6 + [0.6] * 8 + [1.0] * 8 + [1.8] * 5 + [3.0] * 3
    for d in sizes:
        for _ in range(200):
            x, y = rng.uniform(1.5, 20.0), rng.uniform(-11.0, 11.0)
            if math.hypot(x, y) < CLEAR_RADIUS + d:
                continue
            if all(math.hypot(x - cx, y - cy) > (d + cd) / 2 + 0.6 for cx, cy, cd in craters):
                break
        craters.append((x, y, d))
        crater(f, x, y, d, depth=0.25 * d, rim=0.06 * d)
    f.z = soften(f.z, 1)
    flatten_spawn(f)
    uri = write_mesh_model(
        'TerrainCraterField', f, 'Craters from 0.3 to 3 m across, depth a quarter of diameter.', 2)
    z0 = float(f.z.min())
    listing = '\n'.join(f'    ({x:5.2f}, {y:6.2f})  d {d:.1f} m' for x, y, d in craters)
    # Machine-readable list for the benchmark's negative-obstacle check.
    with open(os.path.join(PKG, 'models', 'TerrainCraterField', 'craters.yaml'), 'w') as fh:
        fh.write('# GENERATED by tools/make_terrain_worlds.py. Crater centres (m, world frame),\n'
                 '# rim diameters (m) and depths below the surroundings (m).\n')
        fh.write('craters:\n')
        for x, y, d in craters:
            fh.write(f'  - {{x: {x:.3f}, y: {y:.3f}, diameter: {d:.2f}, depth: {0.25 * d:.3f}}}\n')
    comment = f"""  Craters for negative-obstacle detection. Spawn at the origin, craters ahead in x 1.5 to 20,
  y -11 to 11. Each is a bowl a quarter of its diameter deep with a rim 6 percent of its
  diameter high. Every crater from 0.6 m up is wider than the wheel track (0.436 m).
  Centres and diameters:
{listing}"""
    write_world('crater_field', world('crater_field', comment,
                                      [ground_plane(z0 - 0.5), mesh_terrain_model(uri)]))


def make_rock_field():
    rng = random.Random(SEED + 2)
    rocks = []
    # Dense small rocks everywhere, larger ones sparser. Visible heights straddle step_max
    # (0.10 m in rover_physical.yaml), so some are traversable and some are not.
    rocks += scatter_rocks(rng, 160, (2.0, 20.0), (-8.0, 8.0), lambda x, y: 0.0,
                           size_min=0.06, size_max=0.20, h_min=0.4, h_max=0.8)
    big = scatter_rocks(rng, 50, (3.0, 20.0), (-8.0, 8.0), lambda x, y: 0.0,
                        size_min=0.25, size_max=0.60, h_min=0.4, h_max=0.9)
    rocks += [r.replace('name="rock_', 'name="rock_big_') for r in big]
    comment = """  Rocks for step and clearance checks on flat ground. Spawn at the origin, rocks from x 2
  to 20, y -8 to 8: 160 small (6 to 20 cm wide) and 50 large (25 to 60 cm wide), boxes and
  half-buried ellipsoids. Visible heights straddle step_max (0.10 m)."""
    write_world('rock_field', world('rock_field', comment, [ground_plane(0.0)] + rocks))


def make_urc_mix():
    nrng = np.random.default_rng(SEED + 3)
    rng = random.Random(SEED + 3)
    f = Field(1025, 20.0, 0.0)                  # 51.2 m square, x -5.6..45.6, y -25.6..25.6
    hills(f, nrng, amplitude=0.35, wavelength=12.0)
    add_ramp_lane(f, 12.0, width=3.0, x0=10.0, angle_deg=15.0, height=1.0, plateau=4.0)
    add_ramp_lane(f, -12.0, width=3.0, x0=18.0, angle_deg=25.0, height=0.8, plateau=3.0)
    add_sidehill(f, x0=28.0, x1=36.0, y_low=4.0, y_high=7.0, angle_deg=18.0)
    craters = []
    for d in [0.5] * 8 + [1.2] * 6 + [2.5] * 4:
        for _ in range(200):
            x, y = rng.uniform(4.0, 44.0), rng.uniform(-22.0, 22.0)
            if all(math.hypot(x - cx, y - cy) > (d + cd) / 2 + 1.0 for cx, cy, cd in craters):
                break
        craters.append((x, y, d))
        crater(f, x, y, d, depth=0.25 * d, rim=0.06 * d)
    f.z = soften(f.z, 2)
    flatten_spawn(f)

    def height_at(x, y):
        j = int(round((x - (f.cx - f.size / 2)) / RES))
        i = int(round(((f.cy + f.size / 2) - y) / RES))
        return float(f.z[min(max(i, 0), f.n - 1), min(max(j, 0), f.n - 1)])

    rocks = scatter_rocks(rng, 140, (3.0, 44.0), (-22.0, 22.0), height_at,
                          size_min=0.08, size_max=0.45, h_min=0.4, h_max=0.8)
    # 0.2 m mesh over 51 m keeps the mesh near 130k triangles. Craters from 0.5 m wide are 2.5
    # cells across or more, and the rocks on top are primitives at full detail.
    uri = write_mesh_model(
        'TerrainUrcMix', f, 'URC-like mixed terrain: hills, ramps, a sidehill, craters.', 4)
    z0 = float(f.z.min())
    comment = """  URC-like mixed terrain over 51 x 51 m for long goal sets. Spawn at the origin.
  Gentle hills (about 0.35 m over 12 m wavelengths), a 15 degree ramp lane at y 12 (x 10 on),
  a 25 degree ramp lane at y -12 (x 18 on), an 18 degree sidehill at y 4..7 (full slope x 28 to 36),
  18 craters from 0.5 to 2.5 m across and 140 rocks resting on the surface."""
    write_world('urc_mix', world('urc_mix', comment,
                                 [ground_plane(z0 - 0.5), mesh_terrain_model(uri)] + rocks))


def main():
    make_slopes()
    make_crater_field()
    make_rock_field()
    make_urc_mix()


if __name__ == '__main__':
    main()
