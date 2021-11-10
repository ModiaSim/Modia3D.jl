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

stopTime = 2.0
tolerance = 1e-8
requiredFinalStates = [-0.45515799147473335, 0.05676993769652407, 1.178459912041914, -0.10966302200496965, -0.367365395565088, 0.2903115082177484, 8.759390647284498, -1.3086676277009546, -2.341163220615624, -0.10934417069415246, -1.1736634425032262, -5.6404159425069835]
simulate!(bouncingEllipsoid, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingEllipsoid, ["free.r" "free.rot"; "free.v" "free.w"], figure=1)

end
