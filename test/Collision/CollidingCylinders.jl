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
finalStates = [-0.2999999999999967, -0.7912943154390237, 0.014315906053445, 7.725668052326849e-15, -1.6713198396188602, -1.8252487229313217, 5.311537520022936, 1.0381964675069876e-6, -1.5707962640759698, 2.767601750702146e-6, -1.3060907741482186e-5, -12.479881489610486, -3.0996699092797686e-14, -0.7912167560286282, 0.014397570081453145, -7.552548923863435e-14, -1.6710737093647792, -1.8251003048684926, 3.7406783834915656, -6.519240667683276e-6, 3.6325770690369996e-7, 12.47924271317714, -1.6674546147162458e-5, -8.182725678178327e-5, 0.30000000000000043, -0.7913065574286353, 0.014305776909455472, 1.3838494635067089e-15, -1.6713658613040683, -1.8252727806110998, 5.311432537636687, -1.3602784282928635e-7, -2.92643161263909e-6, 12.479463956421872, 3.6747807896700926e-5, -8.123645641276482e-6]
simulate!(CollidingCylinders, stopTime=stopTime, interval=interval, tolerance=tolerance, log=true, logStates=true, logEvents=true, requiredFinalStates=finalStates)

@usingModiaPlot
plot(CollidingCylinders, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                                    ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
