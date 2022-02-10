module BouncingBeamsSimulation

using Modia3D

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

BouncingBeams = Model(
    boxHeigth = 0.1,
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   visualizeFrames=false,
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
    beamX = Object3D(feature=Solid(shape=Beam(axis=1, length=1.0, width=0.4, thickness=0.2),
                                   visualMaterial=vmatRed,
                                   solidMaterial="DryWood",
                                   massProperties=MassPropertiesFromShape(),
                                   collision=true)),
    coSysX = Object3D(parent=:beamX, feature=Visual(shape=CoordinateSystem(length=0.5))),
    jointX = FreeMotion(obj1=:frameX, obj2=:beamX,
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[0.0, -50*u"°", 0.0]),
                        v=Var(init=[0.0, 2.0, 0.0]),
                        w=Var(init=[2.0, 0.0, 0.0])),
    frameY = Object3D(parent=:world,
                      translation=:[0.0, 0.0, 1.0],
                      rotation=:[90*u"°", 90*u"°", 0.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    beamY = Object3D(feature=Solid(shape=Beam(axis=2, length=1.0, width=0.4, thickness=0.2),
                                   visualMaterial=vmatGreen,
                                   solidMaterial="DryWood",
                                   massProperties=MassPropertiesFromShape(),
                                   collision=true)),
    coSysY = Object3D(parent=:beamY, feature=Visual(shape=CoordinateSystem(length=0.5))),
    jointY = FreeMotion(obj1=:frameY, obj2=:beamY,
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[0.0, 0.0, -50*u"°"]),
                        v=Var(init=[0.0, 0.0, 2.0]),
                        w=Var(init=[0.0, 2.0, 0.0])),
    frameZ = Object3D(parent=:world,
                      translation=:[1.0, 0.0, 1.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    beamZ = Object3D(feature=Solid(shape=Beam(axis=3, length=1.0, width=0.4, thickness=0.2),
                                   visualMaterial=vmatBlue,
                                   solidMaterial="DryWood",
                                   massProperties=MassPropertiesFromShape(),
                                   collision=true)),
    coSysZ = Object3D(parent=:beamZ, feature=Visual(shape=CoordinateSystem(length=0.5))),
    jointZ = FreeMotion(obj1=:frameZ, obj2=:beamZ,
                        r=Var(init=[0.0, 0.0, 0.0]),
                        rot=Var(init=[-50*u"°", 0.0, 0.0]),
                        v=Var(init=[2.0, 0.0, 0.0]),
                        w=Var(init=[0.0, 0.0, 2.0]))
)

bouncingBeams = @instantiateModel(buildModia3D(BouncingBeams), unitless=true, log=false, logStateSelection=false, logCode=false)

#@show bouncingBeams.parameterExpressions
#@show bouncingBeams.parameters

stopTime = 1.2
tolerance = 1e-8
requiredFinalStates = [-0.8527742342910635, 1.6578017942923853, 0.04577174265530922, -0.7904903307190998, 0.3784209895248903, -0.048541407443668916, 4.511726495865782, 1.371932459718465, -1.510024797043685, -7.155505879183188, -0.6205909597968676, 0.6202303171080142, 0.04600259422130081, -0.8603712066366127, 1.6591872146181508, -0.05012483182230522, -0.8502027335229523, 0.39228915465700764, 3.3084796917299557, 0.1357058823264399, -1.6011728205254234, 0.5652834075330013, -7.593556822874329, -0.55350211235096, 1.6580899516035195, 0.04573950567885933, -0.8547165339765472, 0.38182577225339254, -0.048923116390466426, -0.8057409130731289, -1.559903565016631, 0.140455404024548, 3.3294946971080854, -0.6029565330774422, 0.6048964881953146, -7.269193867332278]
simulate!(bouncingBeams, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingBeams, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                     ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
