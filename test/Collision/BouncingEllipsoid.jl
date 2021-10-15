module BouncingEllipsoidSimulation

using ModiaLang
import Modia3D
using  Modia3D.ModiaInterface

BouncingEllipsoid = Model(
    boxHeigth = 0.1,
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox = true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0,-boxHeigth/2,0.0],
                      feature=Solid(shape=Box(lengthX=4.0, lengthY=:boxHeigth, lengthZ=3.0),
                                    visualMaterial=VisualMaterial(color="DarkGreen", transparency=0.5),
                                    solidMaterial="Steel",
                                    collision=true)),
    ellipsoid = Object3D(feature=Solid(shape=Ellipsoid(lengthX=0.1, lengthY=0.2, lengthZ=0.3),
                                       visualMaterial=VisualMaterial(color="Blue"),
                                       solidMaterial="Steel",
                                       collision=true)),
    free = FreeMotion(obj1=:world, obj2=:ellipsoid, r=Var(init=[0.0, 1.0, 0.0]), w=Var(init=[5.0, 0.0, -2.0]))
)

bouncingEllipsoid = @instantiateModel(buildModia3D(BouncingEllipsoid), unitless=true, log=false, logStateSelection=false, logCode=false)

#@show bouncingEllipsoid.parameterExpressions
#@show bouncingEllipsoid.parameters

stopTime = 2.5
tolerance = 1e-8
requiredFinalStates = [-0.8744428143376034, -2.74018215396437, 2.1503500723592053, -0.6683638079010135, -7.522180010264275, 0.8169663199279682, 19.75817247007153, -0.4677263079531673, 0.611944583399547, 7.509405738564828, -0.15529356395511615, -0.1565617242522753]
simulate!(bouncingEllipsoid, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingEllipsoid, ["free.r" "free.rot"; "free.v" "free.w"], figure=1)

end
