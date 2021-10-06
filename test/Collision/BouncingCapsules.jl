module BouncingCapsulesSimulation

using  ModiaLang
import Modia3D
using  Modia3D.ModiaInterface
using  Unitful

vmatRed   = Modia3D.VisualMaterial(color="Red")
vmatGreen = Modia3D.VisualMaterial(color="Green")
vmatBlue  = Modia3D.VisualMaterial(color="Blue")
vmatGrey  = Modia3D.VisualMaterial(color="Grey", transparency=0.5)

BouncingCapsules = Model(
    boxHeigth = 0.1,
    gravField = UniformGravityField(g=9.81, n=[0, 0, -1]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox = true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false,
                                   animationFile="BouncingCapsules.json")),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, 0.0, -boxHeigth/2],
                      feature=Solid(shape=Box(lengthX=5.0, lengthY=3.0, lengthZ=:boxHeigth),
                                    visualMaterial=vmatGrey,
                                    solidMaterial="Steel",
                                    collision=true)),
    frameX = Object3D(parent=:world,
                      translation=:[-1.0, 0.0, 1.0],
                      rotation=:[-90*u"°", 0.0, -90*u"°"],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    capsuleX = Object3D(feature=Solid(shape=Capsule(axis=1, diameter=0.4, length=1.0),
                                      visualMaterial=vmatRed,
                                      solidMaterial="Steel",
                                      collision=true)),
    jointX = FreeMotion(obj1=:frameX, obj2=:capsuleX,
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[0.0, -60*u"°", 0.0]),
                        v=Var(init=[0.0, 1.0, 0.0])),
    frameY = Object3D(parent=:world,
                      translation=:[0.0, 0.0, 1.0],
                      rotation=:[90*u"°", 90*u"°", 0.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    capsuleY = Object3D(feature=Solid(shape=Capsule(axis=2, diameter=0.4, length=1.0),
                                      visualMaterial=vmatGreen,
                                      solidMaterial="Steel",
                                      collision=true)),
    jointY = FreeMotion(obj1=:frameY, obj2=:capsuleY,
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[0.0, 0.0, -60*u"°"]),
                        v=Var(init=[0.0, 0.0, 1.0])),
    frameZ = Object3D(parent=:world,
                      translation=:[1.0, 0.0, 1.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    capsuleZ = Object3D(feature=Solid(shape=Capsule(axis=3, diameter=0.4, length=1.0),
                                      visualMaterial=vmatBlue,
                                      solidMaterial="Steel",
                                      collision=true)),
    jointZ = FreeMotion(obj1=:frameZ, obj2=:capsuleZ,
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[-60*u"°", 0.0, 0.0]),
                        v=Var(init=[1.0, 0.0, 0.0]))
)

bouncingCapsules = @instantiateModel(buildModia3D(BouncingCapsules), unitless=true, log=false, logStateSelection=false, logCode=false)

#@show bouncingCapsules.parameterExpressions
#@show bouncingCapsules.parameters

stopTime = 1.7
tolerance = 1e-8
requiredFinalStates = [-0.7964020579786201, 1.078911664590739, 0.23035071499249554, -0.11077716509170218, 0.4796475444580211, -0.08226573462808072, 2.944349048428646, -0.22998195076361025, 1.5634033775705642, 2.4243768357473545, 0.1525224150890733, -0.038478509902694046, 0.22862096684511138, -0.7970870633627486, 1.080611113333237, -0.06458397074339108, -0.11429374835354875, 0.4851467302974838, 3.3855692118273724, 0.19162868836235442, -1.6236815609521178, -0.06476968434945787, 2.4344563197987243, 0.25547076791303613, 1.0818397999295593, 0.22945161590171845, -0.7966147351633047, 0.4853706855582012, -0.07080711655835899, -0.09965214358471511, -1.5640102628973194, 0.198677718499869, 3.386466629188658, 0.2183469805758011, -0.05750372064225383, 2.441900102029629]
simulate!(bouncingCapsules, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingCapsules, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                        ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
