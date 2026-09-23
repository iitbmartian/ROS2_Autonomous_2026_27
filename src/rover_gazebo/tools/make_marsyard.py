#!/usr/bin/env python3
"""Generate marsyard.sdf and the regolith texture it needs.

Every other world in this package is untextured: the ground planes carry a flat
<diffuse> colour and husarion_logo.dae has no texture maps at all. That is fine for
suspension and driving tests, which is what those worlds are for, but it makes the
visual half of rover_slam untestable. rgbd_odometry needs 15 tracked features to
initialise and finds about 8 against blank surfaces, and RTAB-Map's loop closure
detector keys on the same features.

The evidence is the link counts rtabmap-info reports. A mapping run on
husarion_world.sdf ends with GlobalClosure 0, only geometry-based LocalSpaceClosure
links. The same drive here closes visually. Do not read the "0 words" line as the
symptom: the vocabulary is simply not persisted to the database, and it reads 0 on both
worlds.

This world exists to give both of them something to look at. Run it again after
changing any constant below:

    python3 make_marsyard.py

Two things are deliberate.

Ground texture alone would not be enough. A forward-facing camera sees the ground at a
grazing angle, so ground texture foreshortens into near-uniform bands exactly where the
features are wanted. The rocks carry the visual load at usable viewing angles, and they
double as ICP geometry, so odom_source:=lidar gets a world that is not a bare plane
either.

The rocks are boxes and ellipsoids rather than a scanned mesh because corners and
silhouette edges are what a corner detector actually keys on, and because a generated
world stays reviewable in a diff. SEED fixes the layout, so two people running this get
the same world.
"""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path

import numpy as np
from PIL import Image

SEED = 20260922

# Ground. The visual is a box rather than a plane because a box's faces carry
# well-defined UVs in every renderer; the collision stays an infinite plane, matching
# the other worlds, so physics behaves identically.
GROUND_SIZE = 60.0
GROUND_THICKNESS = 0.02

# 1024 px over 60 m is 5.9 cm per texel. The camera resolves about 1.5 cm per pixel at
# 5 m, so a texel spans roughly four pixels there: comfortably above the scale a corner
# detector works at, while staying coarse enough not to alias into noise. 2048 was tried
# first and looked no better through a 640x360 camera, at four times the file size.
TEXTURE_PX = 1024

# Rocks. The clear radius keeps the spawn point and its settling area empty, so the
# suspension still beds in on flat ground the way it does in the other worlds.
ROCK_COUNT = 110
ROCK_FIELD = 25.0
CLEAR_RADIUS = 3.5
ROCK_MIN, ROCK_MAX = 0.10, 0.45

# Rocks are deliberately low relative to their footprint. The suspension is built to
# climb a 10 cm ledge, so a field of 0.3 m boulders is not a terrain test, it is a wall
# maze: the first attempt at this world used 0.4-0.9 height ratios and the rover wedged
# itself on the third leg of a square. Wide and flat keeps the silhouette and shading a
# feature detector needs while leaving the yard drivable.
ROCK_HEIGHT_MIN, ROCK_HEIGHT_MAX = 0.22, 0.55


def fractal_noise(size: int, octaves: int, rng: np.random.Generator) -> np.ndarray:
    """Tileable multi-octave value noise in [0, 1].

    Each octave is a small lattice of random values resized up with wraparound, so the
    result tiles seamlessly. np.roll-based blending keeps the seam continuous without
    needing a real Perlin implementation.
    """
    out = np.zeros((size, size), dtype=np.float64)
    amplitude, total = 1.0, 0.0
    for o in range(octaves):
        lattice = 2 ** (o + 2)
        grid = rng.random((lattice, lattice))
        # Wrap the lattice before resizing so the upsampled octave tiles.
        grid = np.vstack([grid, grid[:1]])
        grid = np.hstack([grid, grid[:, :1]])
        img = Image.fromarray((grid * 255).astype(np.uint8)).resize(
            (size + 1, size + 1), Image.BICUBIC)
        out += amplitude * (np.asarray(img, dtype=np.float64)[:size, :size] / 255.0)
        total += amplitude
        amplitude *= 0.5
    out /= total
    return (out - out.min()) / (out.ptp() + 1e-9)


def regolith_texture(path: Path) -> None:
    """Write a tileable regolith albedo map.

    Colour comes from blending two ends of a rust-to-grey ramp by the noise value, then
    adding fine per-pixel grain. The grain matters more than the large-scale colour: it
    is what survives at the pixel scale where feature detectors work.
    """
    rng = np.random.default_rng(SEED)
    base = fractal_noise(TEXTURE_PX, octaves=6, rng=rng)
    grain = rng.random((TEXTURE_PX, TEXTURE_PX)) * 0.16 - 0.08

    # Dark rust to pale dust. Kept away from pure black and white so the camera's
    # auto-exposure has headroom and the image does not clip.
    dark = np.array([88.0, 52.0, 38.0])
    light = np.array([176.0, 146.0, 122.0])

    blend = np.clip(base + grain, 0.0, 1.0)[..., None]
    rgb = dark + (light - dark) * blend

    # Scattered darker pebbles, so the texture is not uniformly self-similar. A detector
    # keys on these far more readily than on smooth noise.
    speck = rng.random((TEXTURE_PX, TEXTURE_PX)) > 0.994
    rgb[speck] *= 0.55

    path.parent.mkdir(parents=True, exist_ok=True)
    img = Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8))
    # Quantised and optimised: this is noise, so a full 24-bit PNG costs several MB
    # in the repo for detail no camera here can resolve.
    img.quantize(colors=128, method=Image.MEDIANCUT).save(path, optimize=True)


def rocks(rng: random.Random) -> list[str]:
    """Place rocks on an annulus around the spawn point, varied in size and shade.

    Varying the albedo matters as much as varying the size. A field of identical objects
    gives the bag-of-words vocabulary many near-duplicate descriptors, which is worse
    than having fewer distinct ones: loop closure starts matching the wrong place.
    """
    out = []
    for i in range(ROCK_COUNT):
        while True:
            x = rng.uniform(-ROCK_FIELD, ROCK_FIELD)
            y = rng.uniform(-ROCK_FIELD, ROCK_FIELD)
            if math.hypot(x, y) > CLEAR_RADIUS:
                break

        sx = rng.uniform(ROCK_MIN, ROCK_MAX)
        sy = sx * rng.uniform(0.6, 1.4)
        sz = sx * rng.uniform(ROCK_HEIGHT_MIN, ROCK_HEIGHT_MAX)
        yaw = rng.uniform(0, 2 * math.pi)
        roll = rng.uniform(-0.25, 0.25)

        shade = rng.uniform(0.28, 0.72)
        warm = rng.uniform(0.85, 1.15)
        r = min(shade * warm, 1.0)
        g = shade * 0.82
        b = shade * 0.70

        if rng.random() < 0.45:
            geom = (f"<ellipsoid><radii>{sx / 2:.3f} {sy / 2:.3f} "
                    f"{sz / 2:.3f}</radii></ellipsoid>")
        else:
            geom = f"<box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"

        out.append(f"""    <model name="rock_{i:03d}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {sz / 2:.3f} {roll:.3f} 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material>
            <ambient>{r * 0.4:.3f} {g * 0.4:.3f} {b * 0.4:.3f} 1</ambient>
            <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""")
    return out


def world(texture_uri: str, rock_models: list[str]) -> str:
    return f"""<?xml version="1.0"?>
<!--
  GENERATED by tools/make_marsyard.py. Edit that, not this.

  A textured, rock-strewn yard. The other worlds are bare on purpose, for suspension
  and driving tests; this one exists so the visual half of rover_slam has something to
  track. odom_source:=visual cannot initialise anywhere else, and RTAB-Map's loop
  closure detector needs the same features, so both are untestable without it.

  Declares its systems explicitly for the same reason the other worlds do: the Sensors
  and Imu systems the rover needs are not in Gazebo's default set, and naming any plugin
  disables the default set, so the core trio goes in too.
-->
<sdf version="1.8">
  <world name="marsyard">

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

    <!-- Lower and warmer than the other worlds' sun, and shadows stay on. Raking light
         is what makes the rocks cast the shadows and shade gradients a feature detector
         keys on; a high flat light washes the yard out. -->
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

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <!-- Collision stays an infinite plane, as in every other world, so driving
             behaves identically off the textured square. -->
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <!-- The visual is a thin box, not a plane: box faces carry well-defined UVs, so
             the albedo map lands predictably. Sunk by its own thickness, the top face
             sits exactly at z=0 against the collision plane. -->
        <visual name="visual">
          <pose>0 0 {-GROUND_THICKNESS / 2:.4f} 0 0 0</pose>
          <geometry>
            <box><size>{GROUND_SIZE} {GROUND_SIZE} {GROUND_THICKNESS}</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.45 0.42 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <pbr>
              <metal>
                <albedo_map>{texture_uri}</albedo_map>
                <roughness>0.95</roughness>
                <metalness>0.0</metalness>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>

{chr(10).join(rock_models)}

  </world>
</sdf>
"""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pkg", type=Path, default=Path(__file__).resolve().parent.parent,
                    help="rover_gazebo package root")
    args = ap.parse_args()

    texture = args.pkg / "models/MarsyardGround/materials/textures/regolith.png"
    regolith_texture(texture)

    # model:// resolves because hooks/resource_paths.dsv.in puts share/rover_gazebo/models
    # on GZ_SIM_RESOURCE_PATH. A relative path would resolve against the world file and
    # break once the package is installed.
    uri = "model://MarsyardGround/materials/textures/regolith.png"

    sdf = world(uri, rocks(random.Random(SEED)))
    out = args.pkg / "worlds/marsyard.sdf"
    out.write_text(sdf)

    print(f"texture -> {texture}  ({texture.stat().st_size // 1024} KB)")
    print(f"world   -> {out}  ({ROCK_COUNT} rocks, seed {SEED})")


if __name__ == "__main__":
    main()
