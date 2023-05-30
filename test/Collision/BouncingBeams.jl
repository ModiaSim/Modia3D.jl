module BouncingBeamsSimulation

using Modia3D

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

velocityX = [0.0, 2.0, 0.0]
velocityY = [0.0, 0.0, 2.0]
velocityZ = [2.0, 0.0, 0.0]
rotationX = [0.0, -50*u"°", 0.0]
rotationY = [0.0, 0.0, -50*u"°"]
rotationZ = [-50*u"°", 0.0, 0.0]
angularVelocityX = Modia3D.resolve1(rotationX, [2.0, 0.0, 0.0])
angularVelocityY = Modia3D.resolve1(rotationY, [0.0, 2.0, 0.0])
angularVelocityZ = Modia3D.resolve1(rotationZ, [0.0, 0.0, 2.0])

BouncingBeams = Model3D(
    boxHeigth = 0.1,
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   visualizeFrames=false,
                                   nominalLength=2.0,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox = true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false,
                                   animationFile="BouncingBeams.json")),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, 0.0, -boxHeigth/2],
                      feature=Solid(shape=Box(lengthX=7.0, lengthY=3.0, lengthZ=:boxHeigth),
                                    visualMaterial=vmatGrey,
                                    solidMaterial="DryWood",
                                    massProperties=MassPropertiesFromShape(),
                                    collision=true)),
    frameX = Object3D(parent=:world,
                      translation=:[-1.0, 0.0, 1.0],
                      rotation=:[-90*u"°", 0.0, -90*u"°"],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    beamX = Object3D(parent=:frameX, fixedToParent=false,
                     velocity=velocityX,
                     rotation=rotationX,
                     angularVelocity=angularVelocityX,
                     feature=Solid(shape=Beam(axis=1, length=1.0, width=0.4, thickness=0.2),
                                   visualMaterial=vmatRed,
                                   solidMaterial="DryWood",
                                   massProperties=MassPropertiesFromShape(),
                                   collision=true)),
    coSysX = Object3D(parent=:beamX, feature=Visual(shape=CoordinateSystem(length=0.5))),
    frameY = Object3D(parent=:world,
                      translation=:[0.0, 0.0, 1.0],
                      rotation=:[90*u"°", 90*u"°", 0.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    beamY = Object3D(parent=:frameY, fixedToParent=false,
                     velocity=velocityY,
                     rotation=rotationY,
                     angularVelocity=angularVelocityY,
                     feature=Solid(shape=Beam(axis=2, length=1.0, width=0.4, thickness=0.2),
                                   visualMaterial=vmatGreen,
                                   solidMaterial="DryWood",
                                   massProperties=MassPropertiesFromShape(),
                                   collision=true)),
    coSysY = Object3D(parent=:beamY, feature=Visual(shape=CoordinateSystem(length=0.5))),
    frameZ = Object3D(parent=:world,
                      translation=:[1.0, 0.0, 1.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    beamZ = Object3D(parent=:frameZ, fixedToParent=false,
                     velocity=velocityZ,
                     rotation=rotationZ,
                     angularVelocity=angularVelocityZ,
                     feature=Solid(shape=Beam(axis=3, length=1.0, width=0.4, thickness=0.2),
                                   visualMaterial=vmatBlue,
                                   solidMaterial="DryWood",
                                   massProperties=MassPropertiesFromShape(),
                                   collision=true)),
    coSysZ = Object3D(parent=:beamZ, feature=Visual(shape=CoordinateSystem(length=0.5))),
)

bouncingBeams = @instantiateModel(BouncingBeams, unitless=true, log=false, logStateSelection=false, logCode=false)

#@show bouncingBeams.parameterExpressions
#@show bouncingBeams.parameters

stopTime = 1.2
tolerance = 1e-8
requiredFinalStates = [-0.8104568745056254, 1.7973874611151963, -0.04182468212042101, -0.6843977605721305, 1.1626826154110008, -0.4579740595633442, 4.49795019668019, -1.2036633128916912, -1.4661864515779968, 0.2786617017723218, 2.521549706077493, 6.929493441107961, -0.04316792788452747, -0.8118074155564607, 1.79878733239027, -0.45837679978147955, -0.7109008517331268, 1.1662784938747761, 5.9164930138891965, -0.3057751186242098, -1.6453594624487566, 6.928260119255759, 0.27808243710345887, 2.5204410835287847, 1.8050541127713922, -0.04525571158341206, -0.8223444493426333, 1.1809620524502702, -0.4635597953934979, -0.8311350825736231, -1.527376219825907, 0.31839879289639594, 5.985690312517948, 2.7395852563643754, 7.247345741619606, 0.2638278534698739]
simulate!(bouncingBeams, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingBeams, [("beamX.translation", "beamY.translation", "beamZ.translation") ("beamX.rotation", "beamY.rotation", "beamZ.rotation")
                     ("beamX.velocity", "beamY.velocity", "beamZ.velocity") ("beamX.angularVelocity", "beamY.angularVelocity", "beamZ.angularVelocity")], figure=1)

end
