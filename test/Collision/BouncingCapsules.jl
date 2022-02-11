module BouncingCapsulesSimulation

using Modia3D

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

stopTime = 1.4
tolerance = 1e-8
requiredFinalStates = [-0.7673604039864523, 0.9516988414284928, 0.24982620243724654, 0.014180822268884124, 0.5036323273267752, -0.18340022474242668, 1.5220127656990843, 1.1890544892808237, 1.4103120626088645, 2.622092435351728, -0.22352345880886096, -0.10544477069267612, 0.2498438864904243, -0.7676017038880056, 0.951656101955695, -0.18458496829981588, 0.013038547274916143, 0.503371330630657, 2.7566146861930125, 0.20558772188486563, -1.552314769388157, -0.10825787299295013, 2.6224532099355535, -0.23084017453690012, 0.9516460970951072, 0.249610396714428, -0.767900681642965, 0.5031767718168185, -0.1856868932825187, 0.017287282775231304, -1.510292025990061, 0.19727749473740247, 2.753891529551364, -0.23790362799901707, -0.11054266322392534, 2.6231546726843544]
simulate!(bouncingCapsules, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingCapsules, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                        ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
