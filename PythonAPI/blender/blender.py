import bpy
import math
import os
from mathutils import Vector

# ============================================================
# 1/5 scale autonomous racing car for CARLA-style use
# Image-inspired toy Jeep / ride-on racing car style
#
# Unit: meter
# Axis:
#   X = forward
#   Y = right
#   Z = up
# Origin:
#   ground center between front/rear axles
# ============================================================

# -----------------------------
# Target specifications
# -----------------------------
REAL_LENGTH = 1.400
REAL_WIDTH = 0.775
REAL_HEIGHT = 0.531

WHEELBASE = 0.724
FRONT_TRACK = 0.675
REAR_TRACK = 0.675

WHEEL_RADIUS = 0.135
WHEEL_WIDTH = 0.100
GROUND_CLEARANCE = 0.130

APPROX_MASS_KG = 15.0
DRIVE_TYPE = "AWD"
STEERING_TYPE = "front_wheel_steering"

# Set these to False while only previewing the model in Blender.
EXPORT_FBX = True
EXPORT_COLLISION_FBX = True
CARLA_VEHICLE_NAME = "car_1_5_awd"
OUTPUT_DIR = "/home/hannibal/carla/PythonAPI/blender"
SKELETAL_FBX_PATH = os.path.join(OUTPUT_DIR, f"SK_{CARLA_VEHICLE_NAME}.fbx")
COLLISION_FBX_PATH = os.path.join(OUTPUT_DIR, f"SM_sc_{CARLA_VEHICLE_NAME}.fbx")

# CARLA wheel bones need center-to-center track positions.
# REAL_WIDTH is the leftmost-to-rightmost vehicle width.
# FRONT_TRACK/REAR_TRACK are wheel-center-to-wheel-center distances.
# With WHEEL_WIDTH=0.100 m, the tire outer width becomes track + wheel_width.
TRACK_WIDTH_IS_CENTER_TO_CENTER = True
FRONT_WHEEL_Y = FRONT_TRACK / 2.0
REAR_WHEEL_Y = REAR_TRACK / 2.0
OUTER_TIRE_WIDTH = max(FRONT_TRACK, REAR_TRACK) + WHEEL_WIDTH
TRACK_WIDTH_MISMATCH = abs(OUTER_TIRE_WIDTH - REAL_WIDTH) > 1e-6
TRACK_EXTENDS_BEYOND_BODY = OUTER_TIRE_WIDTH > REAL_WIDTH

FRONT_X = WHEELBASE / 2.0
REAR_X = -WHEELBASE / 2.0
WHEEL_Z = WHEEL_RADIUS

BODY_HALF_WIDTH = 0.265
FENDER_OUTER_Y = REAL_WIDTH / 2.0 - 0.020

CARLA_BASE_BONE = "Vehicle_Base"
CARLA_WHEEL_BONES = {
    "wheel_front_left": "Wheel_Front_Left",
    "wheel_front_right": "Wheel_Front_Right",
    "wheel_rear_left": "Wheel_Rear_Left",
    "wheel_rear_right": "Wheel_Rear_Right",
}

WHEEL_CENTER_LOCATIONS = {
    "wheel_front_left": (FRONT_X, -FRONT_WHEEL_Y, WHEEL_Z),
    "wheel_front_right": (FRONT_X, FRONT_WHEEL_Y, WHEEL_Z),
    "wheel_rear_left": (REAR_X, -REAR_WHEEL_Y, WHEEL_Z),
    "wheel_rear_right": (REAR_X, REAR_WHEEL_Y, WHEEL_Z),
}

if TRACK_WIDTH_MISMATCH:
    print(
        "WARNING: REAL_WIDTH should match track + wheel width. "
        f"Current outer tire width is {OUTER_TIRE_WIDTH:.3f} m, but "
        f"REAL_WIDTH is {REAL_WIDTH:.3f} m."
    )

# -----------------------------
# Clean scene
# -----------------------------
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete()

bpy.context.scene.unit_settings.system = "METRIC"
bpy.context.scene.unit_settings.scale_length = 1.0

# -----------------------------
# Material helpers
# -----------------------------
def make_mat(name, color, roughness=0.55, metallic=0.0):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True

    nodes = mat.node_tree.nodes
    bsdf = None

    for node in nodes:
        if node.type == "BSDF_PRINCIPLED":
            bsdf = node
            break

    if bsdf is None:
        bsdf = nodes.new(type="ShaderNodeBsdfPrincipled")
        output = None

        for node in nodes:
            if node.type == "OUTPUT_MATERIAL":
                output = node
                break

        if output is None:
            output = nodes.new(type="ShaderNodeOutputMaterial")

        mat.node_tree.links.new(bsdf.outputs["BSDF"], output.inputs["Surface"])

    if "Base Color" in bsdf.inputs:
        bsdf.inputs["Base Color"].default_value = color
    if "Roughness" in bsdf.inputs:
        bsdf.inputs["Roughness"].default_value = roughness
    if "Metallic" in bsdf.inputs:
        bsdf.inputs["Metallic"].default_value = metallic
    if "Alpha" in bsdf.inputs:
        bsdf.inputs["Alpha"].default_value = color[3]

    if color[3] < 1.0:
        mat.blend_method = "BLEND"
        mat.show_transparent_back = True
        if hasattr(mat, "use_screen_refraction"):
            mat.use_screen_refraction = True

    return mat


MAT_BODY_WHITE = make_mat("glossy_warm_white_body", (0.96, 0.94, 0.90, 1), 0.34, 0.0)
MAT_BODY_SHADOW = make_mat("slightly_darker_white_panel", (0.82, 0.80, 0.76, 1), 0.45, 0.0)
MAT_BLACK = make_mat("matte_black_plastic", (0.012, 0.012, 0.011, 1), 0.78, 0.0)
MAT_DARK = make_mat("dark_charcoal_detail", (0.055, 0.055, 0.052, 1), 0.68, 0.0)
MAT_RUBBER = make_mat("soft_black_rubber", (0.004, 0.004, 0.003, 1), 0.92, 0.0)
MAT_SILVER = make_mat("bright_silver_chrome_wheel", (0.78, 0.76, 0.72, 1), 0.22, 0.7)
MAT_RED = make_mat("dark_red_seat_and_brake", (0.55, 0.025, 0.018, 1), 0.46, 0.0)
MAT_GLASS = make_mat("smoked_black_glass", (0.025, 0.030, 0.035, 0.45), 0.18, 0.0)
MAT_HEADLIGHT = make_mat("warm_headlight_lens", (1.0, 0.83, 0.45, 1), 0.20, 0.0)
MAT_TAIL = make_mat("red_tail_light_lens", (0.90, 0.020, 0.010, 1), 0.28, 0.0)
MAT_ORANGE = make_mat("orange_side_marker", (1.0, 0.30, 0.020, 1), 0.36, 0.0)

# -----------------------------
# Geometry helpers
# -----------------------------
def shade_smooth(obj):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    try:
        bpy.ops.object.shade_smooth()
    except Exception:
        pass
    obj.select_set(False)


def add_bevel(obj, amount=0.01, segments=2):
    bevel = obj.modifiers.new("rounded_edges", "BEVEL")
    bevel.width = amount
    bevel.segments = segments
    bevel.profile = 0.5

    normal = obj.modifiers.new("weighted_normals", "WEIGHTED_NORMAL")
    normal.keep_sharp = True
    return obj


def rounded_box(name, loc, dims, mat=None, bevel=0.01, segments=2, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc, rotation=rot)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dims
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

    if mat is not None:
        obj.data.materials.append(mat)

    if bevel > 0:
        add_bevel(obj, bevel, segments)

    return obj


def cylinder_y(name, loc, radius, depth, mat=None, vertices=72, bevel=False):
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=depth,
        location=loc,
        rotation=(math.pi / 2, 0, 0)
    )
    obj = bpy.context.object
    obj.name = name

    if mat is not None:
        obj.data.materials.append(mat)

    shade_smooth(obj)

    if bevel:
        add_bevel(obj, 0.004, 1)

    return obj


def cylinder_z(name, loc, radius, depth, mat=None, vertices=48):
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=depth,
        location=loc
    )
    obj = bpy.context.object
    obj.name = name

    if mat is not None:
        obj.data.materials.append(mat)

    shade_smooth(obj)
    return obj


def torus_y(name, loc, major_radius, minor_radius, mat=None):
    bpy.ops.mesh.primitive_torus_add(
        major_radius=major_radius,
        minor_radius=minor_radius,
        major_segments=96,
        minor_segments=12,
        location=loc,
        rotation=(math.pi / 2, 0, 0)
    )
    obj = bpy.context.object
    obj.name = name

    if mat is not None:
        obj.data.materials.append(mat)

    shade_smooth(obj)
    return obj


def cylinder_between(name, p1, p2, radius, mat=None, vertices=16):
    p1 = Vector(p1)
    p2 = Vector(p2)
    mid = (p1 + p2) / 2.0
    direction = p2 - p1
    length = direction.length

    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices,
        radius=radius,
        depth=length,
        location=mid
    )

    obj = bpy.context.object
    obj.name = name
    obj.rotation_euler = direction.to_track_quat("Z", "Y").to_euler()

    if mat is not None:
        obj.data.materials.append(mat)

    shade_smooth(obj)
    return obj


def extruded_side_profile(name, xz_points, width, mat=None, bevel=0.012):
    verts = []
    faces = []
    half_w = width / 2.0

    for x, z in xz_points:
        verts.append((x, -half_w, z))

    for x, z in xz_points:
        verts.append((x, half_w, z))

    n = len(xz_points)

    faces.append(tuple(range(n)))
    faces.append(tuple(range(n, 2 * n))[::-1])

    for i in range(n):
        j = (i + 1) % n
        faces.append((i, j, j + n, i + n))

    mesh = bpy.data.meshes.new(name + "_mesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)

    if mat is not None:
        obj.data.materials.append(mat)

    if bevel > 0:
        add_bevel(obj, bevel, 2)

    return obj


def arch_fender(name, cx, cy, cz, inner_r, outer_r, depth_y, mat=None, start_deg=18, end_deg=162, steps=32):
    verts = []
    faces = []

    y0 = cy - depth_y / 2.0
    y1 = cy + depth_y / 2.0

    angles = [
        math.radians(start_deg + (end_deg - start_deg) * i / steps)
        for i in range(steps + 1)
    ]

    for y in (y0, y1):
        for r in (outer_r, inner_r):
            for a in angles:
                verts.append((cx + r * math.cos(a), y, cz + r * math.sin(a)))

    count = steps + 1
    outer0 = 0
    inner0 = count
    outer1 = count * 2
    inner1 = count * 3

    for i in range(steps):
        faces.append((outer0 + i, outer0 + i + 1, outer1 + i + 1, outer1 + i))
        faces.append((inner0 + i + 1, inner0 + i, inner1 + i, inner1 + i + 1))
        faces.append((outer0 + i, inner0 + i, inner0 + i + 1, outer0 + i + 1))
        faces.append((outer1 + i + 1, inner1 + i + 1, inner1 + i, outer1 + i))

    faces.append((outer0, outer1, inner1, inner0))
    faces.append((outer0 + steps, inner0 + steps, inner1 + steps, outer1 + steps))

    mesh = bpy.data.meshes.new(name + "_mesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)

    if mat is not None:
        obj.data.materials.append(mat)

    add_bevel(obj, 0.004, 1)
    return obj

def add_3d_text(name, text_str, loc, rot, scale, mat=None, extrude=0.01):
    bpy.ops.object.text_add(location=loc, rotation=rot)
    obj = bpy.context.object
    obj.name = name
    obj.data.body = text_str
    obj.data.align_x = 'CENTER'
    obj.data.align_y = 'CENTER'
    obj.data.extrude = extrude
    
    # 텍스트를 MESH로 변환하여 익스포트 가능하게 함
    bpy.ops.object.convert(target='MESH')
    
    obj.scale = scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    
    if mat is not None:
        obj.data.materials.append(mat)
        
    return obj


def add_wheel_spokes(name, cx, cy, cz, outside_sign, spoke_count=10):
    outer_side_y = cy + outside_sign * (WHEEL_WIDTH / 2.0 + 0.005)

    cylinder_y(
        name + "_chrome_outer_ring",
        (cx, outer_side_y, cz),
        0.082,
        0.012,
        MAT_SILVER,
        72,
        True
    )

    cylinder_y(
        name + "_dark_inner_disc",
        (cx, outer_side_y + outside_sign * 0.003, cz),
        0.058,
        0.010,
        MAT_DARK,
        72,
        True
    )

    cylinder_y(
        name + "_center_cap",
        (cx, outer_side_y + outside_sign * 0.009, cz),
        0.021,
        0.010,
        MAT_SILVER,
        48,
        True
    )

    for i in range(spoke_count):
        a = 2 * math.pi * i / spoke_count
        r_mid = 0.050

        spoke = rounded_box(
            f"{name}_chrome_spoke_{i:02d}",
            (
                cx + math.cos(a) * r_mid,
                outer_side_y + outside_sign * 0.011,
                cz + math.sin(a) * r_mid
            ),
            (0.080, 0.010, 0.008),
            MAT_SILVER,
            0.002,
            1
        )

        spoke.rotation_euler[1] = -a


def add_wheel(name, x, y, outside_sign):
    tire = cylinder_y(
        name + "_tire",
        (x, y, WHEEL_Z),
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        MAT_RUBBER,
        96,
        True
    )

    cylinder_y(
        name + "_silver_rim_base",
        (x, y, WHEEL_Z),
        0.088,
        WHEEL_WIDTH + 0.004,
        MAT_SILVER,
        72,
        True
    )

    cylinder_y(
        name + "_black_inner_rim",
        (x, y, WHEEL_Z),
        0.060,
        WHEEL_WIDTH + 0.010,
        MAT_DARK,
        72,
        True
    )

    torus_y(
        name + "_rounded_tire_sidewall",
        (x, y, WHEEL_Z),
        WHEEL_RADIUS - 0.010,
        0.007,
        MAT_RUBBER
    )

    for idx in range(12):
        a = 2.0 * math.pi * idx / 12.0
        block = rounded_box(
            f"{name}_subtle_tread_{idx}",
            (x + math.cos(a) * 0.020, y, WHEEL_Z + math.sin(a) * 0.020),
            (0.012, WHEEL_WIDTH + 0.014, 0.022),
            MAT_DARK,
            0.002,
            1
        )
        block.rotation_euler[1] = -a + math.radians(12 if idx % 2 == 0 else -12)

    add_wheel_spokes(name, x, y, WHEEL_Z, outside_sign)

    empty = bpy.data.objects.new(name + "_center_empty", None)
    empty.empty_display_type = "SPHERE"
    empty.empty_display_size = 0.045
    empty.location = (x, y, WHEEL_Z)
    empty["wheel_radius_m"] = WHEEL_RADIUS
    empty["wheel_width_m"] = WHEEL_WIDTH
    bpy.context.collection.objects.link(empty)

    return tire


def is_vehicle_mesh(obj):
    return obj.type == "MESH" and not obj.get("carla_collision_helper")


def wheel_bone_for_object(obj):
    for wheel_prefix, bone_name in CARLA_WHEEL_BONES.items():
        if obj.name.startswith(wheel_prefix):
            return bone_name
    return CARLA_BASE_BONE


def assign_all_vertices_to_bone(obj, bone_name):
    if obj.type != "MESH":
        return

    obj.vertex_groups.clear()
    group = obj.vertex_groups.new(name=bone_name)

    if obj.data.vertices:
        group.add(range(len(obj.data.vertices)), 1.0, "ADD")


def create_carla_armature():
    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.armature_add(enter_editmode=True, location=(0, 0, 0))

    armature = bpy.context.object
    armature.name = "Armature_" + CARLA_VEHICLE_NAME
    armature.data.name = "Skeleton_" + CARLA_VEHICLE_NAME
    armature.show_in_front = True
    armature["carla_vehicle_name"] = CARLA_VEHICLE_NAME
    armature["approx_mass_kg"] = APPROX_MASS_KG
    armature["drive_type"] = DRIVE_TYPE
    armature["steering"] = STEERING_TYPE

    bones = armature.data.edit_bones
    base_bone = bones[0]
    base_bone.name = CARLA_BASE_BONE
    base_bone.head = (0.0, 0.0, 0.0)
    base_bone.tail = (0.0, 0.0, 0.25)
    base_bone.roll = 0.0

    for wheel_prefix, bone_name in CARLA_WHEEL_BONES.items():
        x, y, z = WHEEL_CENTER_LOCATIONS[wheel_prefix]
        wheel_bone = bones.new(bone_name)
        wheel_bone.head = (x, y, z)
        wheel_bone.tail = (x, y + 0.10, z)
        wheel_bone.parent = base_bone
        wheel_bone.roll = 0.0

    bpy.ops.object.mode_set(mode="OBJECT")
    return armature


def attach_meshes_to_carla_armature(armature, vehicle_root):
    for obj in bpy.context.scene.objects:
        if obj.parent != vehicle_root or not is_vehicle_mesh(obj):
            continue

        bone_name = wheel_bone_for_object(obj)
        assign_all_vertices_to_bone(obj, bone_name)

        obj["carla_skeletal_export"] = True

        modifier = obj.modifiers.new("carla_vehicle_armature", "ARMATURE")
        modifier.object = armature

        world_matrix = obj.matrix_world.copy()
        obj.parent = armature
        obj.matrix_world = world_matrix


def add_collision_box(name, loc, dims):
    obj = rounded_box(name, loc, dims, None, 0.0, 0)
    obj.display_type = "WIRE"
    obj.hide_render = True
    obj["carla_collision_helper"] = True
    return obj


def add_collision_wheel(name, loc):
    obj = cylinder_y(
        name,
        loc,
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        None,
        vertices=16,
        bevel=False
    )
    obj.display_type = "WIRE"
    obj.hide_render = True
    obj["carla_collision_helper"] = True
    return obj


def create_carla_collision_helpers():
    collision_objects = []

    body_height = max(0.08, REAL_HEIGHT - GROUND_CLEARANCE - 0.05)
    body_center_z = GROUND_CLEARANCE + body_height / 2.0
    body_width = min(REAL_WIDTH * 0.72, OUTER_TIRE_WIDTH - WHEEL_WIDTH)

    collision_objects.append(
        add_collision_box(
            "SM_sc_" + CARLA_VEHICLE_NAME + "_body",
            (0.0, 0.0, body_center_z),
            (REAL_LENGTH * 0.88, body_width, body_height)
        )
    )

    collision_objects.append(
        add_collision_box(
            "SM_sc_" + CARLA_VEHICLE_NAME + "_front_bumper",
            (REAL_LENGTH / 2.0 - 0.055, 0.0, 0.205),
            (0.11, min(REAL_WIDTH, OUTER_TIRE_WIDTH), 0.12)
        )
    )

    collision_objects.append(
        add_collision_box(
            "SM_sc_" + CARLA_VEHICLE_NAME + "_rear_bumper",
            (-REAL_LENGTH / 2.0 + 0.055, 0.0, 0.205),
            (0.11, min(REAL_WIDTH, OUTER_TIRE_WIDTH), 0.12)
        )
    )

    for wheel_prefix, loc in WHEEL_CENTER_LOCATIONS.items():
        collision_objects.append(
            add_collision_wheel("SM_sc_" + CARLA_VEHICLE_NAME + "_" + wheel_prefix, loc)
        )

    for obj in collision_objects:
        obj.hide_viewport = True

    return collision_objects


def select_objects(objects):
    bpy.ops.object.select_all(action="DESELECT")

    for obj in objects:
        obj.hide_viewport = False
        obj.hide_set(False)
        obj.select_set(True)

    if objects:
        bpy.context.view_layer.objects.active = objects[0]


def export_carla_skeletal_fbx(armature):
    export_objects = [armature]

    for obj in bpy.context.scene.objects:
        if obj.get("carla_skeletal_export"):
            export_objects.append(obj)

    select_objects(export_objects)

    bpy.ops.export_scene.fbx(
        filepath=SKELETAL_FBX_PATH,
        use_selection=True,
        apply_unit_scale=True,
        bake_space_transform=False,
        object_types={"ARMATURE", "MESH"},
        mesh_smooth_type="FACE",
        add_leaf_bones=False,
        bake_anim=False,
        axis_forward="X",
        axis_up="Z"
    )


def export_carla_collision_fbx(collision_objects):
    select_objects(collision_objects)

    bpy.ops.export_scene.fbx(
        filepath=COLLISION_FBX_PATH,
        use_selection=True,
        apply_unit_scale=True,
        bake_space_transform=False,
        object_types={"MESH"},
        mesh_smooth_type="FACE",
        axis_forward="X",
        axis_up="Z"
    )


# -----------------------------
# Main white body silhouette
# -----------------------------
body_profile = [
    (0.700, 0.155),
    (0.698, 0.252),
    (0.660, 0.292),
    (0.560, 0.308),
    (0.460, 0.342),
    (0.210, 0.358),
    (0.120, 0.390),
    (0.035, 0.385),
    (-0.030, 0.352),
    (-0.355, 0.350),
    (-0.435, 0.392),
    (-0.585, 0.385),
    (-0.690, 0.315),
    (-0.700, 0.155)
]

extruded_side_profile(
    "single_piece_boxy_white_jeep_body_shell",
    body_profile,
    BODY_HALF_WIDTH * 2.0,
    MAT_BODY_WHITE,
    0.020
)

rounded_box(
    "black_underbody_chassis_visible_below_body",
    (0.000, 0.000, 0.145),
    (1.250, 0.455, 0.055),
    MAT_BLACK,
    0.018,
    3
)

# hood shape and raised hood ridges
hood = rounded_box(
    "short_sloped_front_hood",
    (0.440, 0.000, 0.355),
    (0.405, 0.488, 0.046),
    MAT_BODY_WHITE,
    0.018,
    3,
    rot=(0, math.radians(6), 0)
)

rounded_box(
    "left_hood_raised_ridge",
    (0.430, -0.125, 0.374),
    (0.330, 0.025, 0.014),
    MAT_BODY_WHITE,
    0.006,
    1,
    rot=(0, math.radians(6), 0)
)

rounded_box(
    "right_hood_raised_ridge",
    (0.430, 0.125, 0.374),
    (0.330, 0.025, 0.014),
    MAT_BODY_WHITE,
    0.006,
    1,
    rot=(0, math.radians(6), 0)
)

rounded_box(
    "thin_black_hood_air_slot",
    (0.580, 0.000, 0.360),
    (0.110, 0.245, 0.012),
    MAT_BLACK,
    0.005,
    1,
    rot=(0, math.radians(6), 0)
)


# rear flat deck
rounded_box(
    "flat_rear_white_deck",
    (-0.445, 0.000, 0.363),
    (0.330, 0.500, 0.052),
    MAT_BODY_WHITE,
    0.016,
    3
)

# black cockpit tub and side rail
rounded_box(
    "open_black_cockpit_tub",
    (-0.120, 0.000, 0.365),
    (0.390, 0.420, 0.065),
    MAT_BLACK,
    0.018,
    3
)

rounded_box(
    "left_black_cockpit_side_rail",
    (-0.130, -0.233, 0.402),
    (0.470, 0.035, 0.040),
    MAT_BLACK,
    0.010,
    2
)

rounded_box(
    "right_black_cockpit_side_rail",
    (-0.130, 0.233, 0.402),
    (0.470, 0.035, 0.040),
    MAT_BLACK,
    0.010,
    2
)

rounded_box(
    "black_dashboard_block",
    (0.122, 0.000, 0.402),
    (0.100, 0.420, 0.072),
    MAT_BLACK,
    0.012,
    2
)


windshield = rounded_box(
    "small_smoked_windshield_panel",
    (0.152, 0.000, 0.458),
    (0.018, 0.330, 0.112),
    MAT_GLASS,
    0.006,
    1,
    rot=(0, math.radians(-18), 0)
)

# red racing seat
rounded_box(
    "red_seat_bottom_cushion",
    (-0.215, 0.000, 0.387),
    (0.160, 0.218, 0.055),
    MAT_RED,
    0.018,
    3
)

seat_back = rounded_box(
    "red_high_back_bucket_seat",
    (-0.292, 0.000, 0.462),
    (0.064, 0.215, 0.160),
    MAT_RED,
    0.018,
    3,
    rot=(0, math.radians(-12), 0)
)

rounded_box(
    "rounded_red_headrest_above_seat",
    (-0.340, 0.000, 0.540),
    (0.070, 0.190, 0.055),
    MAT_RED,
    0.020,
    3,
    rot=(0, math.radians(-12), 0)
)

for side_name, y in [("left", -0.060), ("right", 0.060)]:
    strap = rounded_box(
        f"{side_name}_black_seat_harness_strap",
        (-0.300, y, 0.488),
        (0.020, 0.018, 0.150),
        MAT_BLACK,
        0.004,
        1,
        rot=(0, math.radians(-12), 0)
    )
    strap.rotation_euler[2] = math.radians(8 if y < 0 else -8)


# roll bar behind seat
cylinder_between("left_black_roll_bar", (-0.405, -0.165, 0.370), (-0.435, -0.165, 0.522), 0.012, MAT_BLACK, 16)
cylinder_between("right_black_roll_bar", (-0.405, 0.165, 0.370), (-0.435, 0.165, 0.522), 0.012, MAT_BLACK, 16)
cylinder_between("top_black_roll_bar", (-0.435, -0.165, 0.522), (-0.435, 0.165, 0.522), 0.012, MAT_BLACK, 16)

# -----------------------------
# Front and rear details
# -----------------------------
rounded_box(
    "flat_black_front_bumper",
    (0.665, 0.000, 0.205),
    (0.070, 0.700, 0.078),
    MAT_BLACK,
    0.018,
    3
)

for side_name, side_sign in [("left", -1), ("right", 1)]:
    rounded_box(
        f"front_{side_name}_chunky_bumper_corner_pod",
        (0.640, side_sign * 0.355, 0.188),
        (0.128, 0.080, 0.120),
        MAT_BLACK,
        0.016,
        3
    )

rounded_box(
    "flat_black_rear_bumper",
    (-0.665, 0.000, 0.205),
    (0.070, 0.690, 0.078),
    MAT_BLACK,
    0.018,
    3
)

# 후면 중앙에 돌출된 양각 로고(DOLBAT) 추가
add_3d_text(
    "rear_dolbat_logo",
    "DOLBAT",
    loc=(-0.705, 0.0, 0.265),
    rot=(math.pi / 2, 0, -math.pi / 2),
    scale=(0.06, 0.06, 0.06),
    mat=MAT_DARK,
    extrude=0.015
)

rounded_box(
    "wraparound_black_front_grille_and_lamp_surround",
    (0.692, 0.000, 0.298),
    (0.020, 0.620, 0.104),
    MAT_BLACK,
    0.014,
    3
)

rounded_box(
    "recessed_dark_center_grille_panel",
    (0.706, 0.000, 0.296),
    (0.010, 0.318, 0.080),
    MAT_DARK,
    0.006,
    1
)

rounded_box(
    "front_grille_top_wrap_lip",
    (0.711, 0.000, 0.345),
    (0.012, 0.590, 0.018),
    MAT_BLACK,
    0.006,
    1
)

rounded_box(
    "front_grille_bottom_wrap_lip",
    (0.711, 0.000, 0.251),
    (0.012, 0.590, 0.018),
    MAT_BLACK,
    0.006,
    1
)

for side_name, y in [("left", -0.302), ("right", 0.302)]:
    rounded_box(
        f"front_{side_name}_outer_grille_end_cap",
        (0.711, y, 0.298),
        (0.012, 0.026, 0.090),
        MAT_BLACK,
        0.006,
        1
    )

for i, y in enumerate([-0.120, -0.080, -0.040, 0.000, 0.040, 0.080, 0.120]):
    rounded_box(
        f"front_grille_vertical_slot_{i}",
        (0.714, y, 0.296),
        (0.010, 0.016, 0.083),
        MAT_BLACK,
        0.003,
        1
    )

for side_name, y in [("left", -0.236), ("right", 0.236)]:
    rounded_box(
        f"front_{side_name}_black_headlight_housing",
        (0.708, y, 0.300),
        (0.012, 0.105, 0.070),
        MAT_DARK,
        0.012,
        2
    )

    rounded_box(
        f"front_{side_name}_warm_headlight_lens",
        (0.718, y, 0.302),
        (0.010, 0.070, 0.042),
        MAT_HEADLIGHT,
        0.008,
        2
    )

    rounded_box(
        f"front_{side_name}_inner_grille_to_lamp_bridge",
        (0.716, y * 0.78, 0.300),
        (0.010, 0.044, 0.074),
        MAT_BLACK,
        0.005,
        1
    )

    rounded_box(
        f"front_{side_name}_orange_side_marker",
        (0.704, y * 0.92, 0.248),
        (0.010, 0.030, 0.018),
        MAT_ORANGE,
        0.004,
        1
    )

    rounded_box(
        f"rear_{side_name}_black_tail_housing",
        (-0.697, y, 0.295),
        (0.020, 0.075, 0.058),
        MAT_BLACK,
        0.008,
        2
    )

    rounded_box(
        f"rear_{side_name}_red_tail_light",
        (-0.710, y, 0.295),
        (0.010, 0.052, 0.040),
        MAT_TAIL,
        0.006,
        1
    )

rounded_box(
    "front_lower_black_skid_plate",
    (0.645, 0.000, 0.132),
    (0.125, 0.420, 0.030),
    MAT_BLACK,
    0.010,
    2,
    rot=(0, math.radians(-6), 0)
)

rounded_box(
    "front_silver_skid_plate_face",
    (0.710, 0.000, 0.115),
    (0.014, 0.340, 0.070),
    MAT_SILVER,
    0.004,
    1,
    rot=(0, math.radians(35), 0)
)

for i, y in enumerate([-0.135, -0.090, -0.045, 0.000, 0.045, 0.090, 0.135]):
    rounded_box(
        f"front_skid_plate_black_vertical_cutout_{i}",
        (0.720, y, 0.112),
        (0.010, 0.018, 0.058),
        MAT_BLACK,
        0.002,
        1,
        rot=(0, math.radians(35), 0)
    )

# -----------------------------
# Side panels, handles, door lines
# -----------------------------
for side_name, side_sign in [("left", -1), ("right", 1)]:
    y_surface = side_sign * (BODY_HALF_WIDTH + 0.006)

    rounded_box(
        f"{side_name}_large_white_door_panel",
        (-0.085, y_surface, 0.260),
        (0.315, 0.010, 0.135),
        MAT_BODY_WHITE,
        0.008,
        1
    )

    rounded_box(
        f"{side_name}_door_lower_shadow_line",
        (-0.085, y_surface + side_sign * 0.006, 0.190),
        (0.300, 0.006, 0.012),
        MAT_BODY_SHADOW,
        0.002,
        1
    )

    rounded_box(
        f"{side_name}_front_vertical_door_cut",
        (0.075, y_surface + side_sign * 0.007, 0.260),
        (0.010, 0.006, 0.130),
        MAT_BODY_SHADOW,
        0.002,
        1
    )

    rounded_box(
        f"{side_name}_rear_vertical_door_cut",
        (-0.240, y_surface + side_sign * 0.007, 0.260),
        (0.010, 0.006, 0.120),
        MAT_BODY_SHADOW,
        0.002,
        1
    )

    rounded_box(
        f"{side_name}_black_door_handle",
        (0.048, y_surface + side_sign * 0.014, 0.305),
        (0.060, 0.017, 0.020),
        MAT_BLACK,
        0.006,
        1
    )

    rounded_box(
        f"{side_name}_rear_small_black_door_handle",
        (-0.278, y_surface + side_sign * 0.014, 0.314),
        (0.056, 0.017, 0.020),
        MAT_BLACK,
        0.006,
        1
    )

    rounded_box(
        f"{side_name}_front_small_side_trim",
        (0.285, y_surface + side_sign * 0.014, 0.322),
        (0.072, 0.015, 0.022),
        MAT_BLACK,
        0.005,
        1
    )

    rounded_box(
        f"{side_name}_slanted_lower_door_sculpt_line",
        (-0.165, y_surface + side_sign * 0.008, 0.235),
        (0.195, 0.006, 0.014),
        MAT_BODY_SHADOW,
        0.002,
        1,
        rot=(0, 0, math.radians(12 * side_sign))
    )

# -----------------------------
# Fender flares and running boards
# -----------------------------
for x, axle_name in [(FRONT_X, "front"), (REAR_X, "rear")]:
    for side_name, side_sign in [("left", -1), ("right", 1)]:
        y_center = side_sign * FENDER_OUTER_Y

        rounded_box(
            f"{axle_name}_{side_name}_flat_top_black_fender",
            (x, y_center, 0.298),
            (0.365, 0.108, 0.056),
            MAT_BLACK,
            0.018,
            3
        )

        arch_fender(
            f"{axle_name}_{side_name}_arched_black_fender_lip",
            x,
            y_center,
            WHEEL_Z,
            WHEEL_RADIUS + 0.010,
            WHEEL_RADIUS + 0.062,
            0.095,
            MAT_BLACK,
            17,
            163,
            34
        )

        for groove_idx, dx in enumerate([-0.115, -0.058, 0.000, 0.058, 0.115]):
            rounded_box(
                f"{axle_name}_{side_name}_fender_top_groove_{groove_idx}",
                (x + dx, y_center + side_sign * 0.010, 0.330),
                (0.030, 0.076, 0.010),
                MAT_DARK,
                0.002,
                1,
                rot=(0, 0, 0)
            )


for side_name, side_sign in [("left", -1), ("right", 1)]:
    rounded_box(
        f"{side_name}_black_running_board",
        (-0.030, side_sign * 0.337, 0.178),
        (0.640, 0.060, 0.052),
        MAT_BLACK,
        0.016,
        3
    )

# -----------------------------
# Wheels
# -----------------------------
add_wheel("wheel_front_left", FRONT_X, -FRONT_WHEEL_Y, -1)
add_wheel("wheel_front_right", FRONT_X, FRONT_WHEEL_Y, 1)
add_wheel("wheel_rear_left", REAR_X, -REAR_WHEEL_Y, -1)
add_wheel("wheel_rear_right", REAR_X, REAR_WHEEL_Y, 1)

# -----------------------------
# Suspension visual parts
# -----------------------------
for axle_name, x in [("front", FRONT_X), ("rear", REAR_X)]:
    for side_name, side_sign in [("left", -1), ("right", 1)]:
        y = side_sign * FRONT_WHEEL_Y

        cylinder_between(
            f"{axle_name}_{side_name}_silver_shock_absorber",
            (x - 0.035, y - side_sign * 0.005, WHEEL_Z + 0.020),
            (x - 0.010, y - side_sign * 0.050, 0.312),
            0.007,
            MAT_SILVER,
            16
        )

        cylinder_between(
            f"{axle_name}_{side_name}_black_front_lower_arm",
            (x - 0.070, y, WHEEL_Z - 0.015),
            (x - 0.160, side_sign * 0.245, 0.143),
            0.006,
            MAT_BLACK,
            12
        )

        cylinder_between(
            f"{axle_name}_{side_name}_black_rear_lower_arm",
            (x + 0.070, y, WHEEL_Z - 0.015),
            (x + 0.160, side_sign * 0.245, 0.143),
            0.006,
            MAT_BLACK,
            12
        )



rounded_box(
    "dashboard_small_screen",
    (0.132, 0.065, 0.415),
    (0.010, 0.075, 0.038),
    MAT_GLASS,
    0.004,
    1
)

rounded_box(
    "front_autonomous_camera_bar",
    (0.610, 0.000, 0.315),
    (0.045, 0.215, 0.030),
    MAT_BLACK,
    0.006,
    1
)

rounded_box(
    "front_left_camera_lens",
    (0.638, -0.055, 0.315),
    (0.010, 0.035, 0.020),
    MAT_GLASS,
    0.004,
    1
)

rounded_box(
    "front_right_camera_lens",
    (0.638, 0.055, 0.315),
    (0.010, 0.035, 0.020),
    MAT_GLASS,
    0.004,
    1
)

# -----------------------------
# Root empty and metadata
# -----------------------------
vehicle_root = bpy.data.objects.new("vehicle_root_center_ground", None)
vehicle_root.empty_display_type = "ARROWS"
vehicle_root.empty_display_size = 0.18
vehicle_root.location = (0, 0, 0)
bpy.context.collection.objects.link(vehicle_root)

vehicle_root["real_length_m"] = REAL_LENGTH
vehicle_root["real_width_m"] = REAL_WIDTH
vehicle_root["real_height_m"] = REAL_HEIGHT
vehicle_root["wheelbase_m"] = WHEELBASE
vehicle_root["front_track_m"] = FRONT_TRACK
vehicle_root["rear_track_m"] = REAR_TRACK
vehicle_root["wheel_radius_m"] = WHEEL_RADIUS
vehicle_root["wheel_width_m"] = WHEEL_WIDTH
vehicle_root["ground_clearance_m"] = GROUND_CLEARANCE
vehicle_root["approx_mass_kg"] = APPROX_MASS_KG
vehicle_root["drive_type"] = DRIVE_TYPE
vehicle_root["steering"] = STEERING_TYPE
vehicle_root["carla_base_bone"] = CARLA_BASE_BONE
vehicle_root["track_width_is_center_to_center"] = TRACK_WIDTH_IS_CENTER_TO_CENTER
vehicle_root["real_width_definition"] = "leftmost_to_rightmost_body_width"
vehicle_root["track_width_definition"] = "wheel_center_to_wheel_center"
vehicle_root["outer_tire_width_m"] = OUTER_TIRE_WIDTH
vehicle_root["track_extends_beyond_body"] = TRACK_EXTENDS_BEYOND_BODY
vehicle_root["note"] = "visual model inspired by attached white ride-on Jeep image"

bbox = rounded_box(
    "hidden_reference_bounding_box_exact_1p4_0p775_0p531",
    (0, 0, REAL_HEIGHT / 2.0),
    (REAL_LENGTH, REAL_WIDTH, REAL_HEIGHT),
    None,
    0,
    0
)
bbox.display_type = "WIRE"
bbox.hide_viewport = True
bbox.hide_render = True

for obj in bpy.context.scene.objects:
    if obj.name not in [vehicle_root.name, bbox.name]:
        obj.parent = vehicle_root

carla_armature = create_carla_armature()
attach_meshes_to_carla_armature(carla_armature, vehicle_root)
carla_collision_objects = create_carla_collision_helpers()

# -----------------------------
# Preview camera and light
# -----------------------------
bpy.ops.object.light_add(type="AREA", location=(1.7, -2.1, 2.3))
light = bpy.context.object
light.name = "large_softbox_preview_light"
light.data.energy = 520
light.data.size = 4.0

bpy.ops.object.camera_add(
    location=(2.10, -2.30, 1.05),
    rotation=(math.radians(62), 0, math.radians(43))
)
camera = bpy.context.object
camera.name = "preview_camera_side_front"
camera.data.lens = 45
bpy.context.scene.camera = camera

# -----------------------------
# Optional export
# -----------------------------
if EXPORT_FBX:
    export_carla_skeletal_fbx(carla_armature)

if EXPORT_COLLISION_FBX:
    export_carla_collision_fbx(carla_collision_objects)

os.makedirs(OUTPUT_DIR, exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUTPUT_DIR, "car_generated.blend"))

print("DONE: image-inspired 1/5 scale autonomous racing car generated.")
print("Target size: 1.400m L x 0.775m W x 0.531m H")
print("Wheelbase:", WHEELBASE, "m")
print("Wheel radius:", WHEEL_RADIUS, "m")
print("CARLA skeletal FBX path:", SKELETAL_FBX_PATH)
print("CARLA collision FBX path:", COLLISION_FBX_PATH)
