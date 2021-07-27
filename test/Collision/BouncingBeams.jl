module BouncingBeamsSimulation

using  ModiaLang
import Modia3D
using  Modia3D.ModiaInterface
using  Unitful

vmatRed   = Modia3D.VisualMaterial(color="Red")
vmatGreen = Modia3D.VisualMaterial(color="Green")
vmatBlue  = Modia3D.VisualMaterial(color="Blue")
vmatGrey  = Modia3D.VisualMaterial(color="Grey", transparency=0.5)

BouncingBeams = Model(
    boxHeigth = 0.1,
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   gap=0.2,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, 0.0, -boxHeigth/2],
                      feature=Solid(shape=Box(lengthX=5.0, lengthY=3.0, lengthZ=:boxHeigth),
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
requiredFinalStates = [-0.8823695490633129, 1.665391474522192, 0.04799938397293286, -1.0204351804781866, 0.43467213487238104, -0.05300623034434308, 4.594448165765738, 1.48458962376636, -1.5821822875254883, -8.616696059760667, -0.38077326260409977, 0.4606687533750722, 0.04448291563173141, -0.8635203337074655, 1.6627077970537474, -0.049200694422998424, -0.8870648971101465, 0.40291237892093623, 3.2801065817877686, 0.1345412028139165, -1.597965173809505, 0.541296269597139, -7.6554365382890746, -0.5338918826110308, 1.9235100250337558, 0.10167700504201196, -1.3991207821301785, 1.5242500783672102, 0.07788755980354922, -3.4667039460822022, -2.7882122968930343, 0.12494117610295302, 4.331872438738044, 1.9861239477070278, -2.898970671344765, 1.0516923739870323]
simulate!(bouncingBeams, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingBeams, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                     ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
