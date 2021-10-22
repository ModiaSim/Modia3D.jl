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
requiredFinalStates = [-0.43409492566620994, 0.05911549571356261, 1.183808741479434, -0.03556004192988097, -0.0028383819889030424, 0.09518649127390906, 10.484235603768825, -1.3383426159657037, -0.6851377375679972, 0.7837914595545629, -0.223523500397426, -0.8825794053946234]
simulate!(bouncingEllipsoid, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingEllipsoid, ["free.r" "free.rot"; "free.v" "free.w"], figure=1)

end
