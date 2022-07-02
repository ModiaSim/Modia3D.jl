module BouncingCapsulesSimulation

using Modia3D

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

BouncingCapsules = Model3D(
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
    capsuleX = Object3D(parent=:frameX, fixedToParent=false,
                        rotation=[0.0, -60*u"°", 0.0],
                        velocity=[0.0, 1.0, 0.0],
                        feature=Solid(shape=Capsule(axis=1, diameter=0.4, length=1.0),
                                      visualMaterial=vmatRed,
                                      solidMaterial="Steel",
                                      collision=true)),
    frameY = Object3D(parent=:world,
                      translation=:[0.0, 0.0, 1.0],
                      rotation=:[90*u"°", 90*u"°", 0.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    capsuleY = Object3D(parent=:frameY, fixedToParent=false,
                        rotation=[0.0, 0.0, -60*u"°"],
                        velocity=[0.0, 0.0, 1.0],
                        feature=Solid(shape=Capsule(axis=2, diameter=0.4, length=1.0),
                                      visualMaterial=vmatGreen,
                                      solidMaterial="Steel",
                                      collision=true)),
    frameZ = Object3D(parent=:world,
                      translation=:[1.0, 0.0, 1.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    capsuleZ = Object3D(parent=:frameZ, fixedToParent=false,
                        rotation=[-60*u"°", 0.0, 0.0],
                        velocity=[1.0, 0.0, 0.0],
                        feature=Solid(shape=Capsule(axis=3, diameter=0.4, length=1.0),
                                      visualMaterial=vmatBlue,
                                      solidMaterial="Steel",
                                      collision=true)),
)

bouncingCapsules = @instantiateModel(BouncingCapsules, unitless=true, log=false, logStateSelection=false, logCode=false)

#@show bouncingCapsules.parameterExpressions
#@show bouncingCapsules.parameters

stopTime = 1.4
tolerance = 1e-8
requiredFinalStates = [-0.7673603852532035, 0.9516987938720817, 0.24982627222882978, 0.014181604247774255, 0.5036323108091764, -0.18340043302903947, 1.5220130850376044, 1.189054333976943, 1.4103120018571025, 0.1404386744698787, 0.7566171345994552, 2.518780172214569, 0.24984395219388528, -0.7676016983741861, 0.9516561037775035, -0.18458497129918844, 0.013039131889693219, 0.5033713001962383, 2.7566146230019166, 0.20558776690709502, -1.5523147262260408, 2.5177060411206855, 0.140488382214332, 0.7639936864415756, 0.9516460922089696, 0.24961032780512585, -0.7679007020906763, 0.503176747722619, -0.18568702124334946, 0.017287192617502473, -1.5102920379213918, 0.19727751116246142, 2.7538915212516004, 0.771095914174634, 2.5170524258497915, 0.1400597191869728]
simulate!(bouncingCapsules, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingCapsules, [("capsuleX.translation", "capsuleY.translation", "capsuleZ.translation") ("capsuleX.rotation", "capsuleY.rotation", "capsuleZ.rotation")
                        ("capsuleX.velocity", "capsuleY.velocity", "capsuleZ.velocity") ("capsuleX.angularVelocity", "capsuleY.angularVelocity", "capsuleZ.angularVelocity")], figure=1)

end
