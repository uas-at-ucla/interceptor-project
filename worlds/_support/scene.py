"""Shared scene builder for the static and moving plate worlds (meters, Z-up)."""

import math
import random
from dataclasses import dataclass


SEARCH_RADIUS_M = 500.0 * 0.3048
FIELD_SIZE_M = 360.0
IRIS_SPAWN_M = (0.0, 0.0, 0.07)
PLATE_RADIUS_M = 0.15
PLATE_RIM_HEIGHT_M = 0.014
PLATE_BOUND_RADIUS_M = math.hypot(PLATE_RADIUS_M, PLATE_RIM_HEIGHT_M)
SQUARE_SIZE_M = 0.14


@dataclass(frozen=True)
class PlatePlacement:
    position: tuple[float, float, float]
    yaw_deg: float

    @property
    def distance_m(self):
        return math.hypot(
            self.position[0] - IRIS_SPAWN_M[0],
            self.position[1] - IRIS_SPAWN_M[1],
        )


def sample_plate(seed: int, height_m: float = 2.0, radius_m: float = SEARCH_RADIUS_M) -> PlatePlacement:
    """Sample uniformly by area inside a horizontal disk (default: 500 feet).

    Height is the plate center above the ground. The upright plate faces spawn.
    A separate random generator keeps placement independent of grass generation.
    """
    if not math.isfinite(height_m) or height_m <= PLATE_RADIUS_M:
        raise ValueError(f"Plate height must be finite and greater than {PLATE_RADIUS_M} m.")
    if not math.isfinite(radius_m) or radius_m < 0:
        raise ValueError("Spawn radius must be finite and nonnegative.")
    rng = random.Random(seed)
    radius = radius_m * math.sqrt(rng.random())
    angle = rng.uniform(-math.pi, math.pi)
    x = IRIS_SPAWN_M[0] + radius * math.cos(angle)
    y = IRIS_SPAWN_M[1] + radius * math.sin(angle)
    # Rotate local +Z toward spawn, while keeping local +Y upright.
    yaw = math.degrees(math.atan2(IRIS_SPAWN_M[1] - y, IRIS_SPAWN_M[0] - x)) + 90
    return PlatePlacement((x, y, height_m), yaw)


def build_world(stage, placement: PlatePlacement, *, moving: bool = False, field_size_m=FIELD_SIZE_M):
    """Author the environment into an Isaac Sim stage without remote asset dependencies.

    The stock Iris is added separately by the launcher through Pegasus Multirotor.
    The plate's square and tape share its transform. Moving worlds use a kinematic
    rigid body so both rendered geometry and collision follow the authored pose.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

    if not math.isfinite(field_size_m) or field_size_m <= 0:
        raise ValueError("Field size must be finite and greater than zero.")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    prefix = "/World/Grassy"
    UsdGeom.Xform.Define(stage, prefix)

    def material(name, color, vertex_colors=False):
        mat = UsdShade.Material.Define(stage, f"{prefix}/Materials/{name}")
        shader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild("Surface"))
        shader.CreateIdAttr("UsdPreviewSurface")
        diffuse = shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
        diffuse.Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
        if vertex_colors:
            reader = UsdShade.Shader.Define(stage, mat.GetPath().AppendChild("Color"))
            reader.CreateIdAttr("UsdPrimvarReader_float3")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
            diffuse.ConnectToSource(reader.ConnectableAPI(), "result")
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat

    grass = material("Grass", (0.12, 0.24, 0.035), vertex_colors=True)
    paper = material("Paper", (0.89, 0.87, 0.79))
    red = material("RedSquare", (0.8, 0.008, 0.005))
    tape = material("MaskingTape", (0.7, 0.62, 0.40))

    def mesh(name, points, faces, mat, colors=None, color_interpolation=UsdGeom.Tokens.uniform):
        obj = UsdGeom.Mesh.Define(stage, f"{prefix}/{name}")
        obj.CreatePointsAttr(points)
        obj.CreateFaceVertexCountsAttr([len(face) for face in faces])
        obj.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
        obj.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        obj.CreateDoubleSidedAttr(True)
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
        if colors is not None:
            obj.CreateDisplayColorPrimvar(color_interpolation).Set(colors)
        return obj

    # One flat ground collider, with its upper surface exactly at Z=0.
    ground = UsdGeom.Cube.Define(stage, f"{prefix}/Ground")
    ground.CreateSizeAttr(1.0)
    ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.1))
    ground.AddScaleOp().Set(Gf.Vec3f(field_size_m, field_size_m, 0.2))
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.24, 0.035)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    ground_mat = UsdShade.Material.Define(stage, f"{prefix}/Materials/GroundPhysics")
    physics = UsdPhysics.MaterialAPI.Apply(ground_mat.GetPrim())
    physics.CreateStaticFrictionAttr(0.8)
    physics.CreateDynamicFrictionAttr(0.65)
    physics.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(ground_mat, materialPurpose="physics")

    # Small variations in the turf plus short blades provide grass at both scales.
    # Geometry uses its own fixed seed so a target seed changes only the target.
    rng = random.Random(81)
    points, faces, colors = [], [], []
    half = field_size_m / 2
    # Keep the turf mesh and blade count bounded as the field grows.
    grid = [-half + i * field_size_m / 120 for i in range(121)]
    stride = len(grid)
    for x in grid:
        for y in grid:
            points.append((x, y, 0.001))
            shade = rng.uniform(0.8, 1.2)
            colors.append((0.12 * shade, 0.24 * shade, 0.035 * shade))
    for row in range(stride - 1):
        for col in range(stride - 1):
            i = row * stride + col
            faces.append((i, i + stride, i + stride + 1, i + 1))
    mesh("Turf", points, faces, grass, colors, UsdGeom.Tokens.vertex)

    points, faces, colors = [], [], []
    for _ in range(45000):
        x, y = rng.uniform(-half, half), rng.uniform(-half, half)
        if math.hypot(x - IRIS_SPAWN_M[0], y - IRIS_SPAWN_M[1]) < 0.6:
            continue
        height = rng.uniform(0.035, 0.085)
        angle = rng.uniform(0, math.tau)
        for offset in (0.0, math.pi / 2):
            dx, dy = 0.018 * math.cos(angle + offset), 0.018 * math.sin(angle + offset)
            i = len(points)
            points.extend([(x - dx, y - dy, 0.002), (x + dx, y + dy, 0.002),
                           (x + dx, y + dy, height)])
            faces.append((i, i + 1, i + 2))
            shade = rng.uniform(0.85, 1.3)
            colors.append((0.10 * shade, 0.26 * shade, 0.025 * shade))
    mesh("GrassBlades", points, faces, grass, colors)

    plate = UsdGeom.Xform.Define(stage, f"{prefix}/PaperPlate")
    plate.AddTranslateOp().Set(Gf.Vec3d(*placement.position))
    plate.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, placement.yaw_deg))
    if moving:
        body = UsdPhysics.RigidBodyAPI.Apply(plate.GetPrim())
        body.CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(plate.GetPrim()).CreateMassAttr(0.015)
    plate.GetPrim().SetCustomData({
        "marker": "red_square", "diameter_m": 2 * PLATE_RADIUS_M,
        "square_size_m": SQUARE_SIZE_M, "distance_from_spawn_m": placement.distance_m,
    })

    # Closed shallow dish: flat center, raised rim, and a thin paper underside.
    profile = [(0.105, 0.0), (0.13, 0.004), (PLATE_RADIUS_M, PLATE_RIM_HEIGHT_M),
               (PLATE_RADIUS_M, 0.012), (0.13, 0.002), (0.105, -0.002)]
    segments = 96
    points = [(radius * math.cos(math.tau * j / segments),
               radius * math.sin(math.tau * j / segments), z)
              for radius, z in profile for j in range(segments)]
    faces = [tuple(range(segments)), tuple(reversed(range((len(profile) - 1) * segments,
                                                         len(profile) * segments)))]
    for ring in range(len(profile) - 1):
        for j in range(segments):
            nxt = (j + 1) % segments
            faces.append((ring * segments + j, (ring + 1) * segments + j,
                          (ring + 1) * segments + nxt, ring * segments + nxt))
    dish = mesh("PaperPlate/Dish", points, faces, paper)
    UsdPhysics.CollisionAPI.Apply(dish.GetPrim())
    if moving:
        UsdPhysics.MeshCollisionAPI.Apply(dish.GetPrim()).CreateApproximationAttr("convexHull")

    def patch(name, size, position, mat):
        obj = UsdGeom.Cube.Define(stage, f"{prefix}/PaperPlate/{name}")
        obj.CreateSizeAttr(1.0)
        obj.AddTranslateOp().Set(Gf.Vec3d(*position))
        obj.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)

    # Tape sits beneath the square; its exposed ends extend onto the plate.
    for name, sign in (("TapeTop", 1), ("TapeBottom", -1)):
        patch(name, (0.025, 0.024, 0.0002), (0, sign * 0.07, 0.00015), tape)
    patch("RedSquare", (SQUARE_SIZE_M, SQUARE_SIZE_M, 0.0004), (0, 0, 0.0005), red)

    sun = UsdLux.DistantLight.Define(stage, f"{prefix}/Sun")
    sun.CreateIntensityAttr(2200)
    sun.CreateAngleAttr(0.8)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-40, -25, 20))
    sky = UsdLux.DomeLight.Define(stage, f"{prefix}/Sky")
    sky.CreateIntensityAttr(650)
    sky.CreateColorAttr(Gf.Vec3f(0.65, 0.79, 1.0))

    # This inspection camera is mounted to the plate, including in moving worlds.
    camera = UsdGeom.Camera.Define(stage, f"{prefix}/PaperPlate/InspectionCamera")
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(0, 0, 1.6), Gf.Vec3d(0), Gf.Vec3d(0, 1, 0))
    camera.AddTransformOp().Set(view.GetInverse())
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000))

    return plate.GetPath()
