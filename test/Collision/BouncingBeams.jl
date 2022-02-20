module BouncingBeamsSimulation

using Modia3D

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

BouncingBeams = Model3D(
    boxHeigth = 0.1,
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox = true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false,
                                   animationFile="BouncingBeams.json")),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, 0.0, -boxHeigth/2],
                      feature=Solid(shape=Box(lengthX=7.0, lengthY=3.0, lengthZ=:boxHeigth),
                                    visualMaterial=vmatGrey,
                                    solidMaterial="DryWood",
                                    massProperties=MassPropertiesFromShape(),
                                    collision=true)),
    frameX = Object3D(parent=:world,
                      translation=:[-1.0, 0.0, 1.0],
                      rotation=:[-90*u"°", 0.0, -90*u"°"],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    beamX = Object3D(feature=Solid(shape=Beam(axis=1, length=1.0, width=0.4, thickness=0.2),
                                   visualMaterial=vmatRed,
                                   solidMaterial="DryWood",
                                   massProperties=MassPropertiesFromShape(),
                                   collision=true)),
    coSysX = Object3D(parent=:beamX, feature=Visual(shape=CoordinateSystem(length=0.5))),
    jointX = FreeMotion(obj1=:frameX, obj2=:beamX,
                        r=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, 0.0)),
                        rot=Var(init=ModiaBase.SVector{3,Float64}(0.0, -50*u"°", 0.0)),
                        v=Var(init=ModiaBase.SVector{3,Float64}(0.0, 2.0, 0.0)),
                        w=Var(init=ModiaBase.SVector{3,Float64}(2.0, 0.0, 0.0))),
    frameY = Object3D(parent=:world,
                      translation=:[0.0, 0.0, 1.0],
                      rotation=:[90*u"°", 90*u"°", 0.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    beamY = Object3D(feature=Solid(shape=Beam(axis=2, length=1.0, width=0.4, thickness=0.2),
                                   visualMaterial=vmatGreen,
                                   solidMaterial="DryWood",
                                   massProperties=MassPropertiesFromShape(),
                                   collision=true)),
    coSysY = Object3D(parent=:beamY, feature=Visual(shape=CoordinateSystem(length=0.5))),
    jointY = FreeMotion(obj1=:frameY, obj2=:beamY,
                        r=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, 0.0)),
                        rot=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, -50*u"°")),
                        v=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, 2.0)),
                        w=Var(init=ModiaBase.SVector{3,Float64}(0.0, 2.0, 0.0))),
    frameZ = Object3D(parent=:world,
                      translation=:[1.0, 0.0, 1.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    beamZ = Object3D(feature=Solid(shape=Beam(axis=3, length=1.0, width=0.4, thickness=0.2),
                                   visualMaterial=vmatBlue,
                                   solidMaterial="DryWood",
                                   massProperties=MassPropertiesFromShape(),
                                   collision=true)),
    coSysZ = Object3D(parent=:beamZ, feature=Visual(shape=CoordinateSystem(length=0.5))),
    jointZ = FreeMotion(obj1=:frameZ, obj2=:beamZ,
                        r=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, 0.0)),
                        rot=Var(init=ModiaBase.SVector{3,Float64}(-50*u"°", 0.0, 0.0)),
                        v=Var(init=ModiaBase.SVector{3,Float64}(2.0, 0.0, 0.0)),
                        w=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, 2.0)))
)

bouncingBeams = @instantiateModel(BouncingBeams, unitless=true, log=false, logStateSelection=false, logCode=false)

#@show bouncingBeams.parameterExpressions
#@show bouncingBeams.parameters

stopTime = 1.2
tolerance = 1e-8
requiredFinalStates = [-0.8104568745056254, 1.7973874611151963,
-0.04182468212042101, -0.6843977605721305, 1.1626826154110008, -0.4579740595633442, 4.49795019668019, -1.2036633128916912, -1.4661864515779968, 7.374259609209814, 0.25467343019105243, 0.09496510178329644, -0.04316792788452747, -0.8118074155564607, 1.79878733239027, -0.45837679978147955, -0.7109008517331268, 1.1662784938747761, 5.9164930138891965, -0.3057751186242098, -1.6453594624487566, 0.09509462328382953, 7.372754384939963, 0.2531821792141959, 1.8050541127713922, -0.04525571158341206, -0.8223444493426333, 1.1809620524502702, -0.4635597953934979, -0.8311350825736231, -1.527376219825907, 0.31839879289639594, 5.985690312517948, 0.3021665793506058, 0.14598734167247776, 7.745079948909601]
simulate!(bouncingBeams, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingBeams, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                     ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
