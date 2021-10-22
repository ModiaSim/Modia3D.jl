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
requiredFinalStates = [-0.7965194528523131, 1.0792671551455382, 0.22983685936959142, -0.10390048793408056, 0.48141860295478556, -0.07448379639208384, 2.9443682731780765, -0.23188102214268777, 1.563780673048276, 2.4254151374395256, 0.1958844644475377, -0.0490560249743359, 0.22982465810682945, -0.7965134095020894, 1.0792585187268442, -0.07968964907912196, -0.11412082604734528, 0.48040268872703684, 3.3778964700118728, 0.19183631936619613, -1.6238000918371827, -0.042124190323066066, 2.4256013675664465, 0.16691805310264796, 1.0793277350996164, 0.23012461216335986, -0.7967195092572273, 0.4805774128786246, -0.07905360853882375, -0.12951695913362057, -1.5655013024617854, 0.19873967709301704, 3.3726841896709825, 0.17809884320229433, -0.044116122678209496, 2.426033809074548]
simulate!(bouncingCapsules, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingCapsules, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                        ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
