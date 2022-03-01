module InValidCollisionPairingError

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
                                   enableContactDetection=true,
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
                                   solidMaterial="BilliardTable",
                                   collision=true)),
    jointX = FreeMotion(obj1=:frameX, obj2=:coneX,
                        r=Var(init=Modia.SVector{3,Float64}(0.0, 0.0, 0.0)),
                        rot=Var(init=Modia.SVector{3,Float64}(0.0, -60*u"°", 0.0)),
                        v=Var(init=Modia.SVector{3,Float64}(0.0, 1.0, 0.0))),
    frameY = Object3D(parent=:world,
                      translation=:[0.0, -0.5, 1.0],
                      rotation=:[90*u"°", 90*u"°", 0.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneY = Object3D(feature=Solid(shape=Cone(axis=2, diameter=0.4, length=1.0, topDiameter=0.3),
                                   visualMaterial=vmatGreen,
                                   solidMaterial="DryWood",
                                   collision=true)),
    jointY = FreeMotion(obj1=:frameY, obj2=:coneY,
                        r=Var(init=Modia.SVector{3,Float64}(0.0, 0.0, 0.0)),
                        rot=Var(init=Modia.SVector{3,Float64}(0.0, 0.0, -60*u"°")),
                        v=Var(init=Modia.SVector{3,Float64}(0.0, 0.0, 1.0))),
    frameZ = Object3D(parent=:world,
                      translation=:[1.0, -0.5, 1.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneZ = Object3D(feature=Solid(shape=Cone(axis=3, diameter=0.4, length=1.0, topDiameter=0.3),
                                   visualMaterial=vmatBlue,
                                   solidMaterial="DryWood",
                                   collision=true)),
    jointZ = FreeMotion(obj1=:frameZ, obj2=:coneZ,
                        r=Var(init=Modia.SVector{3,Float64}(0.0, 0.0, 0.0)),
                        rot=Var(init=Modia.SVector{3,Float64}(-60*u"°", 0.0, 0.0)),
                        v=Var(init=Modia.SVector{3,Float64}(1.0, 0.0, 0.0)))
)

bouncingCones = @instantiateModel(BouncingCones, unitless=true, log=false, logStateSelection=false, logCode=false)


stopTime = 1.3
tolerance = 1e-8
simulate!(bouncingCones, stopTime=stopTime, tolerance=tolerance)

#@usingModiaPlot
#plot(bouncingCones, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
#                     ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
