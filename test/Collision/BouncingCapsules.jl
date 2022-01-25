module BouncingCapsulesSimulation

using Modia
using  Unitful

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

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
requiredFinalStates = [-0.7965194324923187, 1.0792671324200518, 0.22983676627347815, -0.1039097850692818, 0.4814176273397363, -0.07448832005070478, 2.944368266298714, -0.23188092804556582, 1.5637806069044045, 2.4254147963752337, 0.1958593686673586, -0.049049991414040636, 0.22982554355367993, -0.7965131155676861, 1.0792581962112746, -0.07973508965627857, -0.11418691979366435, 0.48039300578570787, 3.377894872964063, 0.19183612343753909, -1.6238015814380535, -0.0420638591264601, 2.4255988934027917, 0.1666656948426664, 1.0793274621567706, 0.23012495483972084, -0.7967193671718964, 0.4805756438722489, -0.07905877747249435, -0.12952010214850182, -1.5655006683501074, 0.19873974775783254, 3.3726826053720087, 0.17806826424566943, -0.04410873189732083, 2.426030228460745]
simulate!(bouncingCapsules, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingCapsules, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                        ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
