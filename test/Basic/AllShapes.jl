module AllShapesTest

using  ModiaLang
import Modia3D
using  Modia3D.ModiaInterface

filenameTiger = joinpath(Modia3D.path, "objects", "fish", "SiameseTiger0.3ds")
filenameBunny = joinpath(Modia3D.path, "objects", "bunny", "bunny.obj")

# Properties
mat1 = VisualMaterial(color="Green", transparency=0.5)
mat2 = "Red"

font2 = Font(fontFamily="FreeSans", bold=false, italic=false, charSize=0.1,
    color = "LightBlue", transparency=0.0)

# material for massProperties where all MassProperties are defined
m   = 2.0
rCM = [1,2,3]
massDefined = MassProperties(mass=m, centerOfMass=rCM, Ixx=3.0, Iyy=4.0, Izz=4.0, Ixy=6.0, Ixz=7.0, Iyz=9.0)

# mass properties are computed from shape and mass
massAndGeo = MassPropertiesFromShapeAndMass(mass=4.5)

# mass properties are computed from shape and solid material
geomAndSolidMat = MassPropertiesFromShape()

# Objects3D
AllShapes = Model(
    gravField = UniformGravityField(g=9.81, n=[0, 0,-1]),
    world = Object3D(feature=Scene(gravityField=:gravField)),
    worldFrame = Object3D(parent=:world,feature=Visual(shape=CoordinateSystem())),

    # visual elements
    sphere = Object3D(parent=:world, feature=Visual(shape=Sphere(diameter=0.5)),
        translation=[5.5, 0.0, 2.5]),
    ellipsoid = Object3D(parent=:world, feature=Visual(shape =
        Ellipsoid(lengthX=0.6,lengthY=0.9, lengthZ=0.3)), translation=[0.0, 0.0, 2.5]),
    box = Object3D(parent=:world, feature=Visual(shape=Box(lengthX=0.9, lengthY=0.5,
        lengthZ=0.3)), translation=[3.5, 0.0, 2.5]),
    cylinder = Object3D(parent=:world, feature=Visual(shape=Cylinder(diameter=0.5,
        length=0.8)), translation=[1.5, 0.0, 2.5]),
    pipe = Object3D(parent=:world, feature=Visual(shape=Cylinder(diameter=0.5,
        length=0.8, innerDiameter=0.45)), translation=[-4.5, 0.0, 2.5]),
    capsule = Object3D(parent=:world, feature=Visual(shape=Capsule(diameter=0.4,
        length=0.45)), translation=[-1.5, 0.0, 2.5]),
    beam = Object3D(parent=:world, feature=Visual(shape=Beam(axis=1, length=0.4, width=0.5, thickness=0.3)),
        translation=[ -2.6, 0.0, 2.5]),
    cone = Object3D(parent=:world, feature=Visual(shape=Cone(axis=3, diameter=0.3, length=0.7, topDiameter=0.15)),
        translation=[ -3.6, 0.0, 2.5], rotation=Modia3D.rot1(-pi/3)),
    bunny = Object3D(parent=:world, feature=Visual(shape =
        FileMesh(filename = filenameBunny, scale=[0.1, 0.1, 0.1])), translation=[-6.0, 0.0, 2.5]),
    bunnyPartition = Object3D(parent=:world, feature = Visual(shape =
        FileMesh(filename = filenameBunny, scale=[0.1, 0.1, 0.1], convexPartition=true)), translation=[-8.0, 0.0, 2.5]),

    # Solid objects
    sphereSolid = Object3D(parent=:world, feature=Solid(shape=Sphere(diameter=0.5),
        visualMaterial=mat1, solidMaterial="Steel"), translation=[5.5, 0.0, 0.0]),

    ellipsoidSolid = Object3D(
        feature=Solid(shape=Ellipsoid(lengthX=0.6, lengthY=0.9, lengthZ=0.3), visualMaterial=mat1, solidMaterial="DryWood")),
    free = FreeMotion(obj1=:world, obj2=:ellipsoidSolid),

    boxSolid = Object3D(
        feature=Solid(shape=Box(lengthX=0.9, lengthY=0.5, lengthZ=0.3), visualMaterial=mat1, solidMaterial="Steel")),
    fixed = Fix(obj1=:world, obj2=:boxSolid, translation=[3.5, 0.0, 0.0]),

    cylinderSolid = Object3D(parent=:world, feature=Solid(shape=Cylinder(diameter=0.5,
        length=0.8), visualMaterial=mat1, solidMaterial="Steel"), translation=[1.5, 0.0, 0.0]),
    pipeSolid = Object3D(parent=:world, feature=Solid(shape=Cylinder(diameter=0.5,
        length=0.8, innerDiameter=0.45), visualMaterial=mat1, solidMaterial="Steel", massProperties=massDefined), translation=[-4.5, 0.0, 0.0]),

    capsuleSolid = Object3D(parent=:world, feature=Solid(shape=Capsule(diameter=0.4,
        length=0.45), visualMaterial=mat1, massProperties=massAndGeo), translation=[-1.5, 0.0, 0.0]),

    beamSolid = Object3D(parent=:world, feature=Solid(shape=Beam(axis=1, length=0.4, width=0.5, thickness=0.3),visualMaterial=mat1, solidMaterial="Steel", massProperties=geomAndSolidMat), translation =[ -2.6, 0.0, 0.0]),

    coneSolid = Object3D(parent=:world, feature=Solid(shape=Cone(axis=3, diameter=0.3, length=0.7, topDiameter=0.15),
        visualMaterial=mat1, solidMaterial="Steel", collision=true, massProperties = geomAndSolidMat), translation=[ -3.6, 0.0, 0.0], rotation=Modia3D.rot1(-pi/3)),
    bunnySolid = Object3D(parent=:world, feature=Solid(shape =
        FileMesh(filename=filenameBunny, scale=[0.1, 0.1, 0.1]), solidMaterial="Steel", massProperties = geomAndSolidMat ), translation=[-6.0, 0.0, 0.0]),
    bunnyPartitionSolid = Object3D(parent=:world, feature=Solid(shape =
        FileMesh(filename = filenameBunny, scale=[0.1, 0.1, 0.1], convexPartition=true), solidMaterial="Steel", collision=true), translation=[-8.0, 0.0, 0.0]),

    # pure visual objects
    spring    = Object3D(parent=:world, feature=Visual(shape =
        SpringShape(axis=3, length=0.8, diameter=0.3, wireDiameter=0.04, windings=5), visualMaterial=mat2), translation=[3.0, 0.0, -2.5]),
    gearWheel = Object3D(parent=:world, feature=Visual(shape =
        GearWheel(axis=3, diameter=0.5, length=0.8, innerDiameter=0.25, angle=15/180*pi, teeth=16), visualMaterial=mat2), translation=[4.5, 0.0, -2.5]),
    grid      = Object3D(parent=:world, feature=Visual(shape =
        Grid(axis=3, length=1.0, width=0.6, distance=0.05, lineWidth=1.0)), translation=[0.0, 0.0, -2.5]),
    coSystem  = Object3D(parent=:world, feature = Visual(shape =
        CoordinateSystem(length=0.7)), translation=[-1.5, 0.0, -2.5]),
    fileMeshTiger = Object3D(parent=:world, feature=Visual(shape =
        FileMesh(filename=filenameTiger, scale=[4.0, 4.0, 2.5])), translation=[-3.0, 0.0, -2.5]),
    # Text object
    text = Object3D(parent=:world, feature = Visual(shape =
    TextShape(text="Hello world!")), translation=[1.0, 0.0, -2.5]),
)

allShapes = @instantiateModel(buildModia3D(AllShapes), unitless=true, log=false, logStateSelection=false, logCode=false)

simulate!(allShapes, stopTime=0.0, log=false, logStates=false, logEvents=false)

println("... Basics/AllShapes.jl completed.")

end
