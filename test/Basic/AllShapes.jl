module AllShapesTest

using Modia3D

filenameBunny = joinpath(Modia3D.path, "objects", "bunny", "bunny.obj")

# Properties
vmatBlue = VisualMaterial(color="Blue", transparency=0.25)
vmatGreen = VisualMaterial(color="Green", transparency=0.75)
vmatYellow = VisualMaterial(color="Yellow", transparency=0.5)

font2 = Font(fontFamily="FreeSans", bold=false, italic=false, charSize=0.1, color="LightBlue", transparency=0.0)

# material for massProperties where all MassProperties are defined
m   = 2.0
rCM = [1,2,3]
massDefined = MassProperties(mass=m, centerOfMass=rCM, Ixx=3.0, Iyy=4.0, Izz=4.0, Ixy=6.0, Ixz=7.0, Iyz=9.0)

# mass properties are computed from shape and mass
massAndGeo = MassPropertiesFromShapeAndMass(mass=4.5)

# mass properties are computed from shape and solid material
geomAndSolidMat = MassPropertiesFromShape()

# Objects3D
AllShapes = Model3D(
    gravField = UniformGravityField(g=9.81, n=[0, 0, -1]),
    world = Object3D(feature=Scene(gravityField=:gravField)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.7))),

    # solid objects
    solidSphere = Object3D(parent=:world, feature=Solid(shape=
        Sphere(diameter=0.5), visualMaterial=vmatBlue, solidMaterial="Steel"), translation=[-7.0, 0.0, 0.0]),

    solidEllipsoid = Object3D(parent=:world, fixedToParent=false, translation=[-6.0, 0.0, 0.0], feature=Solid(shape=
        Ellipsoid(lengthX=0.6, lengthY=0.9, lengthZ=0.3), visualMaterial=vmatBlue, solidMaterial="DryWood")),

    solidBox = Object3D(feature=Solid(shape=
        Box(lengthX=0.9, lengthY=0.5, lengthZ=0.3), visualMaterial=vmatBlue, solidMaterial="Steel")),
    jointBox = Fix(obj1=:world, obj2=:solidBox, translation=[-5.0, 0.0, 0.0]),

    solidCylinder = Object3D(parent=:world, feature=Solid(shape=
        Cylinder(diameter=0.5, length=0.8), visualMaterial=vmatBlue, solidMaterial="Steel"), translation=[-4.0, 0.0, 0.0]),

    solidPipe = Object3D(parent=:world, feature=Solid(shape=
        Cylinder(diameter=0.5, length=0.8, innerDiameter=0.45), visualMaterial=vmatBlue, solidMaterial="Steel", massProperties=massDefined), translation=[-3.0, 0.0, 0.0]),

    solidCone = Object3D(parent=:world, feature=Solid(shape=
        Cone(axis=3, diameter=0.3, length=0.7), visualMaterial=vmatBlue, solidMaterial="Steel", collision=true, massProperties=geomAndSolidMat), translation=[-2.0, 0.0, 0.0]),

    solidFrustum = Object3D(parent=:world, feature=Solid(shape=
        Cone(axis=3, diameter=0.3, length=0.7, topDiameter=0.15), visualMaterial=vmatBlue, solidMaterial="Steel", collision=true, massProperties=geomAndSolidMat), translation=[-1.0, 0.0, 0.0]),

    solidCapsule = Object3D(parent=:world, feature=Solid(shape=
        Capsule(diameter=0.4, length=0.45), visualMaterial=vmatBlue, massProperties=massAndGeo), translation=[1.0, 0.0, 0.0]),

    solidBeam = Object3D(parent=:world, feature=Solid(shape=
        Beam(axis=1, length=0.4, width=0.5, thickness=0.3), visualMaterial=vmatBlue, solidMaterial="Steel", massProperties=geomAndSolidMat), translation =[2.0, 0.0, 0.0]),

    solidFileMesh = Object3D(parent=:world, feature=Solid(shape=
        FileMesh(filename=filenameBunny, scale=[0.05, 0.05, 0.05]), solidMaterial="Steel", massProperties=geomAndSolidMat), translation=[3.0, 0.0, 0.0]),

    solidFileMeshPartitioned = Object3D(parent=:world, feature=Solid(shape=
        FileMesh(filename=filenameBunny, scale=[0.05, 0.05, 0.05], convexPartition=true), solidMaterial="Steel", collision=true), translation=[4.0, 0.0, 0.0]),

    # visual objects
    visualSphere = Object3D(parent=:world, feature=Visual(shape=
        Sphere(diameter=0.5), visualMaterial=vmatGreen), translation=[-7.0, 0.0, 1.0]),

    visualEllipsoid = Object3D(parent=:world, feature=Visual(shape=
        Ellipsoid(lengthX=0.6, lengthY=0.9, lengthZ=0.3), visualMaterial=vmatGreen), translation=[-6.0, 0.0, 1.0]),

    visualBox = Object3D(parent=:world, feature=Visual(shape=
        Box(lengthX=0.9, lengthY=0.5, lengthZ=0.3), visualMaterial=vmatGreen), translation=[-5.0, 0.0, 1.0]),

    visualCylinder = Object3D(parent=:world, feature=Visual(shape=
        Cylinder(diameter=0.5, length=0.8), visualMaterial=vmatGreen), translation=[-4.0, 0.0, 1.0]),

    visualPipe = Object3D(parent=:world, feature=Visual(shape=
        Cylinder(diameter=0.5, length=0.8, innerDiameter=0.45), visualMaterial=vmatGreen), translation=[-3.0, 0.0, 1.0]),

    visualCone = Object3D(parent=:world, feature=Visual(shape=
        Cone(axis=3, diameter=0.3, length=0.7), visualMaterial=vmatGreen), translation=[-2.0, 0.0, 1.0]),

    visualFrustum = Object3D(parent=:world, feature=Visual(shape=
        Cone(axis=3, diameter=0.3, length=0.7, topDiameter=0.15), visualMaterial=vmatGreen), translation=[-1.0, 0.0, 1.0]),

    visualCapsule = Object3D(parent=:world, feature=Visual(shape=
        Capsule(diameter=0.4, length=0.45), visualMaterial=vmatGreen), translation=[1.0, 0.0, 1.0]),

    visualBeam = Object3D(parent=:world, feature=Visual(shape=
        Beam(axis=1, length=0.4, width=0.5, thickness=0.3), visualMaterial=vmatGreen), translation=[2.0, 0.0, 1.0]),

    visualFileMesh = Object3D(parent=:world, feature=Visual(shape=
        FileMesh(filename=filenameBunny, scale=[0.05, 0.05, 0.05])), translation=[3.0, 0.0, 1.0]),

    visualFileMeshPartitioned = Object3D(parent=:world, feature=Visual(shape=
        FileMesh(filename=filenameBunny, scale=[0.05, 0.05, 0.05], convexPartition=true)), translation=[4.0, 0.0, 1.0]),

    # pure visual objects
    visualCoordinateSystem = Object3D(parent=:world, feature=Visual(shape=
        CoordinateSystem(length=0.7)), translation=[0.0, 0.0, 1.0]),

    visualGrid = Object3D(parent=:world, feature=Visual(shape=
        Grid(axis=3, length=1.0, width=0.6, distance=0.05, lineWidth=1.0)), translation=[5.0, 0.0, 1.0]),

    visualSpring = Object3D(parent=:world, feature=Visual(shape=
        SpringShape(axis=3, length=0.8, diameter=0.3, wireDiameter=0.04, windings=5), visualMaterial=vmatGreen), translation=[6.0, 0.0, 1.0]),

    visualGearWheel = Object3D(parent=:world, feature=Visual(shape=
        GearWheel(axis=3, diameter=0.5, length=0.8, innerDiameter=0.25, angle=15/180*pi, teeth=16), visualMaterial=vmatGreen), translation=[7.0, 0.0, 1.0]),

    visualText = Object3D(parent=:world, feature=Visual(shape=
        TextShape(text="Modia3D", axisAlignment=Modia3D.XZ_Plane, alignment=Modia3D.Left)), translation=[8.0, 0.0, 1.0]),

    # modelica visual objects
    modelicaBox = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=1, lengthX=0.9, lengthY=0.5, lengthZ=0.3), visualMaterial=vmatYellow), translation=[-5.0, 0.0, -1.0]),

    modelicaEllipsoid = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=2, lengthX=0.6, lengthY=0.9, lengthZ=0.3), visualMaterial=vmatYellow), translation=[-6.0, 0.0, -1.0]),

    modelicaCylinder = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=3, lengthX=0.5, lengthY=0.5, lengthZ=0.8), visualMaterial=vmatYellow), translation=[-4.0, 0.0, -1.0]),

    modelicaFrustum = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=4, lengthX=0.3, lengthY=0.3, lengthZ=0.7, extra=[0.15/0.3, 0.0, 0.0]), visualMaterial=vmatYellow), translation=[-1.0, 0.0, -1.0]),

    modelicaCapsule = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=5, lengthX=0.4, lengthY=0.4, lengthZ=0.45), visualMaterial=vmatYellow), translation=[1.0, 0.0, -1.0]),

    modelicaCoordinateSystem = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=6, lengthX=0.7, lengthY=0.7, lengthZ=0.7)), translation=[0.0, 0.0, -1.0]),

    modelicaSpring = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=7, lengthX=0.8, lengthY=0.3, lengthZ=0.3, extra=[5, 0.04/2, 0.0]), visualMaterial=vmatYellow), translation=[6.0, 0.0, -1.0]),

    modelicaGearWheel = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=8, lengthX=0.5, lengthY=0.5, lengthZ=0.8, extra=[0.25/0.5, 16, 15.0]), visualMaterial=vmatYellow), translation=[7.0, 0.0, -1.0]),

    modelicaPipe = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=9, lengthX=0.5, lengthY=0.5, lengthZ=0.8, extra=[0.0, 0.45/0.5, 1.0]), visualMaterial=vmatYellow), translation=[-3.0, 0.0, -1.0]),

    modelicaGrid = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=10, lengthX=1.0, lengthY=0.6, lengthZ=0.0, extra=[0.05, 1.0, 0.0])), translation=[5.0, 0.0, -1.0]),

    modelicaBeam = Object3D(parent=:world, feature=Visual(shape=
        ModelicaShape(type=11, lengthX=0.4, lengthY=0.5, lengthZ=0.3), visualMaterial=vmatYellow), translation=[2.0, 0.0, -1.0])
)

allShapes = @instantiateModel(AllShapes, unitless=true, log=false, logStateSelection=false, logCode=false, FloatType=Float32)



simulate!(allShapes, stopTime=0.0, log=false, logStates=false, logEvents=false)

println("... Basics/AllShapes.jl completed.")

end
