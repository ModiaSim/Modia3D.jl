module BouncingBeamsSimulation

using  ModiaLang
import Modia3D
using  Modia3D.ModiaInterface
using  Unitful

vmatRed   = Modia3D.Shapes.VisualMaterial(color="Red")
vmatGreen = Modia3D.Shapes.VisualMaterial(color="Green")
vmatBlue  = Modia3D.Shapes.VisualMaterial(color="Blue")
vmatGrey  = Modia3D.Shapes.VisualMaterial(color="Grey", transparency=0.5)

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
requiredFinalStates = [-0.8263521589113617, 1.6556214390497919, 0.04317349406175776, -0.519437377848517, 0.3117728241063305, -0.04195225652181148, 4.481871408047987, 1.2472352973128984, -1.4833427135714758, -5.743718568793708, -0.5563464573524693, 0.6775553584977084, 0.04352893572571521, -0.8427016302018774, 1.658821049586814, -0.04462317976858583, -0.7244166216422334, 0.3647498757776394, 3.3819643323506665, 0.13300591418859095, -1.6265447569135538, 0.7048282778118946, -6.382261451514867, -0.7121100652630898, 1.9270674418477596, 0.07151125755482043, -1.5112112210129487, 1.5545851452913053, 0.00981264862243235, -3.785593015594227, -2.6804336155553288, 0.20616084009806535, 5.30872523707291, -1.3074304809968607, -2.7888646900960685, 4.298089432264489]
simulate!(bouncingBeams, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingBeams, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                     ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
