module BouncingConesSimulation

using Modia
using  Unitful

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

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

stopTime = 1.2
tolerance = 1e-8
requiredFinalStates = [-0.7966975786520888, 0.939985832590693, 0.38951809262163817, 0.008283684681695894, 0.6658233515495816, 0.0036360762130771824, 1.5168127712425896, 0.809516683182946, 1.6309255834121132, 3.326610309487532, -0.09442757282865713, 0.1303056174217491, 0.3895556675053645, -0.7966982542294878, 0.939989290934488, 0.0035470907869790593, 0.0092555728573629, 0.6658258352729927, 2.381830853536271, -0.021037308472438263, -1.533543880143283, 0.1309791419088494, 3.326660575823015, -0.09374545764051212, 0.939978985399097, 0.3893676586735939, -0.7967319752002895, 0.6658573796203757, 0.0038700820465069068, 0.005146202892539086, -1.612281246246739, 0.010392303991357257, 2.3818260851771695, -0.09767677033008831, 0.12731399635504168, 3.3267904560140074]
simulate!(bouncingCones, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingCones, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                     ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
