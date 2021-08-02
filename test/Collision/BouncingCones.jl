module BouncingConesSimulation

using  ModiaLang
import Modia3D
using  Modia3D.ModiaInterface
using  Unitful

vmatRed   = Modia3D.VisualMaterial(color="Red")
vmatGreen = Modia3D.VisualMaterial(color="Green")
vmatBlue  = Modia3D.VisualMaterial(color="Blue")
vmatGrey  = Modia3D.VisualMaterial(color="Grey", transparency=0.5)

BouncingCones = Model(
    boxHeigth = 0.1,
    gravField = UniformGravityField(g=9.81, n=[0, 0, -1]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox = true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false,
                                   animationFile="BouncingCones.json")),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, 0.0, -boxHeigth/2],
                      feature=Solid(shape=Box(lengthX=5.0, lengthY=3.0, lengthZ=:boxHeigth),
                                    visualMaterial=vmatGrey,
                                    solidMaterial="DryWood",
                                    collision=true)),
    frameX = Object3D(parent=:world,
                      translation=:[-1.0, -0.5, 1.0],
                      rotation=:[-90*u"°", 0.0, -90*u"°"],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneX = Object3D(feature=Solid(shape=Cone(axis=1, diameter=0.4, length=1.0, topDiameter=0.3),
                                   visualMaterial=vmatRed,
                                   solidMaterial="DryWood",
                                   collision=true)),
    jointX = FreeMotion(obj1=:frameX, obj2=:coneX,
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[0.0, -60*u"°", 0.0]),
                        v=Var(init=[0.0, 1.0, 0.0])),
    frameY = Object3D(parent=:world,
                      translation=:[0.0, -0.5, 1.0],
                      rotation=:[90*u"°", 90*u"°", 0.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneY = Object3D(feature=Solid(shape=Cone(axis=2, diameter=0.4, length=1.0, topDiameter=0.3),
                                   visualMaterial=vmatGreen,
                                   solidMaterial="DryWood",
                                   collision=true)),
    jointY = FreeMotion(obj1=:frameY, obj2=:coneY,
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[0.0, 0.0, -60*u"°"]),
                        v=Var(init=[0.0, 0.0, 1.0])),
    frameZ = Object3D(parent=:world,
                      translation=:[1.0, -0.5, 1.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneZ = Object3D(feature=Solid(shape=Cone(axis=3, diameter=0.4, length=1.0, topDiameter=0.3),
                                   visualMaterial=vmatBlue,
                                   solidMaterial="DryWood",
                                   collision=true)),
    jointZ = FreeMotion(obj1=:frameZ, obj2=:coneZ,
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[-60*u"°", 0.0, 0.0]),
                        v=Var(init=[1.0, 0.0, 0.0]))
)

bouncingCones = @instantiateModel(buildModia3D(BouncingCones), unitless=true, log=false, logStateSelection=false, logCode=false)

#@show bouncingCones.parameterExpressions
#@show bouncingCones.parameters

stopTime = 1.3
tolerance = 1e-8
requiredFinalStates = [-0.8002799390065646, 1.0059964298604027, 0.39039287407921686, -0.025430996181140066, 0.6630234043302788, 0.004309151343939569, 1.4686527860326997, 1.1368340779745558, 1.689846158106513, 3.3149879046571797, -0.06960222124441125, 0.1504261450316327, 0.3903893470590204, -0.8003514923828058, 1.0070415055199, 0.0011832492706785752, -0.055916265529015616, 0.6653193436925052, 2.717877804098592, -0.026017382314878938, -1.5277965562323037, 0.13944370362475916, 3.325930468488853, -0.09546231236269272, 1.0069646410862858, 0.3901947305115851, -0.8003618381020178, 0.6653930682717618, -6.006508633896099e-5, -0.0588692411973017, -1.6206623791265042, -0.006015411115857988, 2.717355681962454, -0.10225978390299145, 0.1363774922811096, 3.3263550311419343]
simulate!(bouncingCones, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingCones, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                     ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
