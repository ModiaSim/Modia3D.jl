module BouncingBeamsSimulation

using  ModiaLang
import Modia3D
using  Modia3D.ModiaInterface
using  Unitful

vmatRed   = Modia3D.VisualMaterial(color="Red")
vmatGreen = Modia3D.VisualMaterial(color="Green")
vmatBlue  = Modia3D.VisualMaterial(color="Blue")
vmatGrey  = Modia3D.VisualMaterial(color="Grey", transparency=0.5)

BouncingBeams = Model(
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
                      feature=Solid(shape=Box(lengthX=5.0, lengthY=3.0, lengthZ=:boxHeigth),
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
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[0.0, -50*u"°", 0.0]),
                        v=Var(init=[0.0, 2.0, 0.0]),
                        w=Var(init=[2.0, 0.0, 0.0])),
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
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[0.0, 0.0, -50*u"°"]),
                        v=Var(init=[0.0, 0.0, 2.0]),
                        w=Var(init=[0.0, 2.0, 0.0])),
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
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[-50*u"°", 0.0, 0.0]),
                        v=Var(init=[2.0, 0.0, 0.0]),
                        w=Var(init=[0.0, 0.0, 2.0]))
)

bouncingBeams = @instantiateModel(buildModia3D(BouncingBeams), unitless=true, log=false, logStateSelection=false, logCode=false)

#@show bouncingBeams.parameterExpressions
#@show bouncingBeams.parameters

stopTime = 1.2
tolerance = 1e-8
requiredFinalStates = [-0.826400892194186, 1.6556305083562353, 0.04317674746456983, -0.5199568390051346, 0.3119042407793094, -0.04195711713927978, 4.481838904053375, 1.247493807035776, -1.4832969419692021, -5.746687727383563, -0.5566452080021305, 0.6775667280019738, 0.0435298686854457, -0.8427114859200519, 1.6588227481877853, -0.044625293755116695, -0.7244946139184326, 0.36476801685087074, 3.3819071930476263, 0.13300805955026176, -1.6265286525951494, 0.7047335685118996, -6.382964356769341, -0.712031979267392, 1.9270934200576517, 0.07152279653829582, -1.511148053521765, 1.5546827037252273, 0.00984007386846095, -3.785373459053888, -2.6804106838976693, 0.2062007270454431, 5.308750966317778, -1.3075780328829656, -2.7886388544514227, 4.298271852663165]
simulate!(bouncingBeams, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingBeams, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                     ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
