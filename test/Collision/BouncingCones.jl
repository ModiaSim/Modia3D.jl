module BouncingConesSimulation

using Modia3D

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

BouncingCones = Model3D(
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
    coneX = Object3D(parent=:frameX, fixedToParent=false,
                     rotation=[0.0, -60*u"°", 0.0],
                     velocity=[0.0, 1.0, 0.0],
                     feature=Solid(shape=Cone(axis=1, diameter=0.4, length=1.0),
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
                     feature=Solid(shape=Cone(axis=2, diameter=0.4, length=1.0),
                                   visualMaterial=vmatGreen,
                                   solidMaterial="DryWood",
                                   collision=true)),
    frameZ = Object3D(parent=:world,
                      translation=:[1.0, -0.5, 1.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneZ = Object3D(parent=:frameZ, fixedToParent=false,
                     rotation=[-60*u"°", 0.0, 0.0],
                     velocity=[1.0, 0.0, 0.0],
                     feature=Solid(shape=Cone(axis=3, diameter=0.4, length=1.0),
                                   visualMaterial=vmatBlue,
                                   solidMaterial="DryWood",
                                   collision=true))
)

bouncingCones = @instantiateModel(BouncingCones, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 1.2
tolerance = 1e-8
requiredFinalStates = [-0.7999311625652963, 1.0550589340109533, 0.8921255200490438, -0.18969494953144356, 0.8841403146779124, 0.43402600857718365, 1.3606906685626896, 1.1282713416507297, 2.0528833436012595, -0.030303266151766442, -1.514825830499711, 4.510758033358663, 0.8916596085646129, -0.800040034923405, 1.0551915260680085, 0.43363424025808023, -0.19001477209520348, 0.8846396171051895, 2.7358742077100384, -0.2906589654151757, -1.4769196629577481, 4.514472763028119, -0.030810996624682587, -1.515427655179665, 1.0551437942348352, 0.8923793589673877, -0.8001535827544871, 0.884868848392877, 0.4342007829869422, -0.1917344341104826, -1.7767415484303393, -0.2277633762477909, 2.698544037765237, -1.521254393123658, 4.516519345054235, -0.03229086240525738]
simulate!(bouncingCones, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingPlotPackage
plot(bouncingCones, [("coneX.translation", "coneY.translation", "coneZ.translation") ("coneX.rotation", "coneY.rotation", "coneZ.rotation")
                     ("coneX.velocity", "coneY.velocity", "coneZ.velocity") ("coneX.angularVelocity", "coneY.angularVelocity", "coneZ.angularVelocity")], figure=1)

end
