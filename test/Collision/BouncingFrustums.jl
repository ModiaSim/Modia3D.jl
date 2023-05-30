module BouncingFrustumsSimulation

using Modia3D

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

BouncingFrustums = Model3D(
    boxHeigth = 0.1,
    gravField = UniformGravityField(g=9.81, n=[0, 0, -1]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false,
                                   nominalLength=2.0,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox = true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false)),
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
    coneX = Object3D(parent=:frameX, fixedToParent=false,
                     rotation=[0.0, -60*u"°", 0.0],
                     velocity=[0.0, 1.0, 0.0],
                     feature=Solid(shape=Cone(axis=1, diameter=0.4, length=1.0, topDiameter=0.3),
                                   visualMaterial=vmatRed,
                                   solidMaterial="DryWood",
                                   collision=true)),
    frameY = Object3D(parent=:world,
                      translation=:[0.0, -0.5, 1.0],
                      rotation=:[90*u"°", 90*u"°", 0.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneY = Object3D(parent=:frameY, fixedToParent=false,
                     rotation=[0.0, 0.0, -60*u"°"],
                     velocity=[0.0, 0.0, 1.0],
                     feature=Solid(shape=Cone(axis=2, diameter=0.4, length=1.0, topDiameter=0.3),
                                   visualMaterial=vmatGreen,
                                   solidMaterial="DryWood",
                                   collision=true)),
    frameZ = Object3D(parent=:world,
                      translation=:[1.0, -0.5, 1.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneZ = Object3D(parent=:frameZ, fixedToParent=false,
                     rotation=[-60*u"°", 0.0, 0.0],
                     velocity=[1.0, 0.0, 0.0],
                     feature=Solid(shape=Cone(axis=3, diameter=0.4, length=1.0, topDiameter=0.3),
                                   visualMaterial=vmatBlue,
                                   solidMaterial="DryWood",
                                   collision=true))
)

bouncingFrustums = @instantiateModel(BouncingFrustums, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 1.2
tolerance = 1e-8
requiredFinalStates = [-0.800383758696937, 0.949741937109897, 0.38422127001602735, 0.07288055494670782, 0.6958162086297855, 0.02928114392694946, 1.5105310059041943, 0.898119482643151, 1.6377172728247884, 0.0249943736329831, -0.19041506193314, 3.4867605703264064, 0.38392633247105096, -0.7999007836143879, 0.9506521787824257, 0.01858528412490112, -0.15332873114484707, 0.6954024470051271, 2.4765087135097246, -0.017901203986858016, -1.53539300618968, 3.49400206910128, 0.03178922643898537, 0.2688280046672574, 0.9502971301756091, 0.3824740603974736, -0.7910626574280535, 0.7004417337114894, 0.02536843290108837, 0.06457785563906525, -1.6290607416066407, 0.007344877012290217, 2.475842546041932, -0.2035283758057457, 3.4990370424861688, -0.023454704613229824]
simulate!(bouncingFrustums, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingFrustums, [("coneX.translation", "coneY.translation", "coneZ.translation") ("coneX.rotation", "coneY.rotation", "coneZ.rotation")
                        ("coneX.velocity", "coneY.velocity", "coneZ.velocity") ("coneX.angularVelocity", "coneY.angularVelocity", "coneZ.angularVelocity")], figure=1)

end
