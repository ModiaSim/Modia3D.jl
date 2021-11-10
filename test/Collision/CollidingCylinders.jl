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
finalStates = [-0.3, -0.7912942238599145, 0.014316028029694703, -4.951823339004666e-26, -1.6713195989803598, -1.8252483603577718, 5.311537425321111, 1.6032315663676118e-13, -1.5707963267948892, 4.090001952987472e-13, -1.977813136643233e-12, -12.479880859047597, 2.9774641985852018e-27, -0.7912164653485692, 0.014398072420987033, -3.11403904939305e-27, -1.671072932445298, -1.8250987920173498, 3.740679216188579, -3.197010426896491e-14, 7.168387035284525e-16, 12.47924435822188, -6.71867832682112e-14, -3.9536089018448937e-13, 0.3, -0.7913065368845195, 0.01430580305471953, 2.43203273029372e-26, -1.6713657986515387, -1.8252727190452591, 5.311432701899439, -2.3971393369822783e-15, -5.143223137587274e-14, 12.47946422427759, 6.344797633530953e-13, -1.2940266587864184e-13]
simulate!(CollidingCylinders, stopTime=stopTime, interval=interval, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=finalStates)

@usingModiaPlot
plot(CollidingCylinders, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                                    ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
