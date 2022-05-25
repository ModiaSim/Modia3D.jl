module CollidingCylindersSimulation

using Modia3D

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

LxTable = 2.0
LyTable = 2.0
LzTable = 0.02
diameterCyl      = 0.1
lengthCyl        = 0.5
innerDiameterCyl = 0.08

collidingCylinders = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   enableContactDetection=true)),
    cylinderX = Object3D(parent=:world, fixedToParent=false,
                         translation=[-0.3, -0.1, 0.5],
                         rotation=[0.0, -90*u"°", -80*u"°"],
                         feature=Solid(shape=Cylinder(axis=1, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatRed,
                                       collision=true)),
    cylinderY = Object3D(parent=:world, fixedToParent=false,
                         translation=[0.0, -0.1, 0.5],
                         rotation=[-10*u"°", 0.0, 0.0],
                         feature=Solid(shape=Cylinder(axis=2, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatGreen,
                                       collision=true)),
    cylinderZ = Object3D(parent=:world, fixedToParent=false,
                         translation=[0.3, -0.1, 0.5],
                         rotation=[80*u"°", 0.0, 0.0],
                         feature=Solid(shape=Cylinder(axis=3, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatBlue,
                                       collision=true)),
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

CollidingCylinders = @instantiateModel(collidingCylinders, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 0.7
interval = 0.001
tolerance = 1e-8
finalStates = [-0.3, -0.7970361906996144, 0.008588032900251187, 1.812580975664453e-23, -1.689651474865682, -1.8362931570189545, 5.311030404991287, 1.7598326328386403e-13, -1.5707963267948886, 4.3735209891956707e-13, -2.1706467601558663e-12, -12.501896978184602, -8.59673448708876e-24, -0.7958169252024361, 0.009907525197720464, 4.6940568471699435e-27, -1.6857839833248573, -1.833831957607084, 3.739172633236702, 7.433636126054145e-13, -3.4753118007399046e-14, 12.49166771329692, 1.8928348395802786e-12, 9.182496637792402e-12, 0.3, -0.797235019007845, 0.008404751485410525, -7.139830646698457e-27, -1.6904048032727732, -1.8367363292526138, 5.309290144499847, -2.3835703797910944e-15, -5.1140206673097534e-14, 12.494972603130112, 6.315585582568632e-13, -1.2837880825135223e-13]
simulate!(CollidingCylinders, stopTime=stopTime, interval=interval, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=finalStates)

@usingModiaPlot
plot(CollidingCylinders, [("cylinderX.translation", "cylinderY.translation", "cylinderZ.translation") ("cylinderX.rotation", "cylinderY.rotation", "cylinderZ.rotation")
                          ("cylinderX.velocity", "cylinderY.velocity", "cylinderZ.velocity") ("cylinderX.angularVelocity", "cylinderY.angularVelocity", "cylinderZ.angularVelocity")], figure=1)

end
