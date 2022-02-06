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
finalStates = [-0.3, -0.791294266958862, 0.014315939807270254, 1.4140917407679343e-22, -1.6713197955240835, -1.825248546396337, 5.311536893824697, 1.6032313112096607e-13, -1.5707963267948892, 4.0900021913041826e-13, -1.9778128832132113e-12, -12.479878969620117, -1.235382560976447e-28, -0.7912141473014349, 0.014393908281690475, -3.1306499139729373e-28, -1.6710648815372404, -1.8251055980240192, 3.740707780374352, -8.407660248040008e-15, 1.6937962402662573e-16, 12.479395737705257, -1.6224145063400506e-14, -1.0380772833456439e-13, 0.3, -0.7913065660977002, 0.014305768987642438, 4.1584081087890855e-27, -1.6713659189025871, -1.8252727838268923, 5.311432402918108, -2.3971395633393076e-15, -5.1432233524106005e-14, 12.479463143727862, 6.344797727935901e-13, -1.2940268558584234e-13]
simulate!(CollidingCylinders, stopTime=stopTime, interval=interval, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=finalStates)

@usingModiaPlot
plot(CollidingCylinders, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                                    ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
