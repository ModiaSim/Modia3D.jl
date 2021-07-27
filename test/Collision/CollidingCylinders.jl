module CollidingCylindersSimulation

using  Unitful
using  ModiaLang
import Modia3D
using  Modia3D.ModiaInterface

vmatRed   = Modia3D.VisualMaterial(color="Red")
vmatGreen = Modia3D.VisualMaterial(color="Green")
vmatBlue  = Modia3D.VisualMaterial(color="Blue")
vmatGrey  = Modia3D.VisualMaterial(color="Grey", transparency=0.5)

LxTable = 2.0
LyTable = 2.0
LzTable = 0.02
diameterCyl      = 0.1
lengthCyl        = 0.5
innerDiameterCyl = 0.08

collidingCylinders = Model(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   enableContactDetection=true)),

    cylinderX = Object3D(feature=Solid(shape=Cylinder(axis=1, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatRed,
                                       collision=true)),
    jointX = FreeMotion(obj1=:world, obj2=:cylinderX,
                        r=Var(init=[-0.3, -0.1, 0.5]),
                        rot=Var(init=[0.0, -90*u"°", -80*u"°"])),

    cylinderY = Object3D(feature=Solid(shape=Cylinder(axis=2, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatGreen,
                                       collision=true)),
    jointY = FreeMotion(obj1=:world, obj2=:cylinderY,
                        r=Var(init=[0.0, -0.1, 0.5]),
                        rot=Var(init=[-10*u"°", 0.0, 0.0])),

    cylinderZ = Object3D(feature=Solid(shape=Cylinder(axis=3, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatBlue,
                                       collision=true)),
    jointZ = FreeMotion(obj1=:world, obj2=:cylinderZ,
                        r=Var(init=[0.3, -0.1, 0.5]),
                        rot=Var(init=[80*u"°", 0.0, 0.0])),

    cylinder2 = Object3D(parent=:world,
                         translation=[0.0, 0.1, 0.2],
                         feature=Solid(shape=Cylinder(axis=1, diameter=diameterCyl, length=1.0),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatGrey,
                                       collision=true)),
    cylinder3 = Object3D(parent=:world,
                         translation=[0.0, -0.1, 0.0],
                         feature=Solid(shape=Cylinder(axis=1, diameter=diameterCyl, length=1.0),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatGrey,
                                       collision=true))
)

CollidingCylinders = @instantiateModel(buildModia3D(collidingCylinders), unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 0.7
interval = 0.001
tolerance = 1e-8
finalStates = [-0.3000014767733773, -1.082193896696148, -0.538807311042302, -3.7592282983652485e-6, -2.5117143103911763, -3.747655431035406, 3.434780596077875, -1.557228662514374, 1.617103948525645, 0.03212608204313396, 0.005364383932787988, 0.5421088990973728, -3.042579138039232e-7, -1.0813130417689532, -0.5361375150624056, -7.007175995563201e-7, -2.5091653603011435, -3.740308350308572, 0.27706268760792585, 0.011027339253852072, -0.0002832776767408359, -0.4592873394400626, 0.021172006600497985, -0.005240358556866595, 0.3000003035109844, -1.0823403796115827, -0.5378963004593084, 8.132044608411955e-7, -2.5121486704670146, -3.7451155263449594, 1.8287413219661708, 0.0005856583646075422, -0.019142491087689967, -0.511322469433515, -0.007676420293499906, -0.04351478584418659]
simulate!(CollidingCylinders, stopTime=stopTime, interval=interval, tolerance=tolerance, log=true, logStates=true, logEvents=true, requiredFinalStates=finalStates)

@usingModiaPlot
plot(CollidingCylinders, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                                    ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
