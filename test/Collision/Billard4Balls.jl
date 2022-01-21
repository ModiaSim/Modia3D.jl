module Billard4Balls

using ModiaLang

import Modia3D
using  Modia3D.ModiaInterface

vmatBalls = Modia3D.VisualMaterial(color="Red", transparency=0.0)
vmatTable = Modia3D.VisualMaterial(color="DarkGreen", transparency=0.5)

diameter = 0.06
radius = diameter/2
distance_balls = sqrt(3)/2*diameter
dist = 0.0001
TableX = 2.24
TableY = 1.12
TableZ = 0.05
LyBox = 0.05
LzBox = 0.05
hz = 7*radius/5
hzz = hz/3

Table = Model(
    table = Object3D(parent=:world,
                     translation=[0.0, 0.0, -TableZ/2],
                     feature=Solid(shape=Box(lengthX=TableX, lengthY=TableY, lengthZ=TableZ),
                                   solidMaterial="BilliardTable",
                                   visualMaterial=vmatTable,
                                   collision=true,
                                   collisionSmoothingRadius=0.001))
)

Cushion = Model(
    cushion11 = Object3D(parent=:world,
                         translation=[TableX/2-LyBox/2, 0.0, hz/2],
                         feature=Solid(shape=Box(lengthX=LyBox, lengthY=TableY, lengthZ=hz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatTable)),
    cushion12 = Object3D(parent=:cushion11,
                         translation=[-LyBox/4, 0.0, hz/2+hzz/2],
                         feature=Solid(shape=Box(lengthX=1.5*LyBox, lengthY=TableY, lengthZ=hzz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatTable,
                                       collision=true)),
    cushion31 = Object3D(parent=:world,
                         translation=[-TableX/2+LyBox/2, 0.0, hz/2],
                         feature=Solid(shape=Box(lengthX=LyBox, lengthY=TableY, lengthZ=hz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatTable)),
    cushion32 = Object3D(parent=:cushion31,
                         translation=[LyBox/4, 0.0, hz/2+hzz/2],
                         feature=Solid(shape=Box(lengthX=1.5*LyBox, lengthY=TableY, lengthZ=hzz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatTable,
                                       collision=true)),
    cushion21 = Object3D(parent=:world,
                         translation=[0.0, TableY/2-LyBox/2, hz/2],
                         feature=Solid(shape=Box(lengthX=TableX, lengthY=LyBox, lengthZ=hz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatTable)),
    cushion22 = Object3D(parent=:cushion21,
                         translation=[0.0, -LyBox/4, hz/2+hzz/2],
                         feature=Solid(shape=Box(lengthX=TableX, lengthY=1.5*LyBox, lengthZ=hzz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatTable,
                                       collision=true)),
    cushion41 = Object3D(parent=:world,
                         translation=[0.0, -TableY/2+LyBox/2, hz/2],
                         feature=Solid(shape=Box(lengthX=TableX, lengthY=LyBox, lengthZ=hz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatTable)),
    cushion42 = Object3D(parent=:cushion41,
                         translation=[0.0, LyBox/4, hz/2+hzz/2],
                         feature=Solid(shape=Box(lengthX=TableX, lengthY=1.5*LyBox, lengthZ=hzz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatTable,
                                       collision=true))
)

Ball = Model(
    ball = Object3D(feature=Solid(shape=Sphere(diameter=diameter),
                                  solidMaterial="BilliardBall",
                                  visualMaterial=vmatBalls,
                                  collision=true))
)

Billard = Model(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                          enableContactDetection=true,
                                          maximumContactDamping=1000.0)),
    table = Table,
    cushion = Cushion,
    ball0 = Ball,
    joint0 = FreeMotion(obj1=:world, obj2=:(ball0.ball),
                        r=Var(init=[-0.8, -0.1, diameter/2]),
                        v=Var(init=[3.0, 0.1, 0.0]),
                        rot=Var(init=[pi/2, 0.0, 0.0])),
    ball1 = Ball,
    joint1 = FreeMotion(obj1=:world, obj2=:(ball1.ball),
                        r=Var(init=[TableX/6, 0.0, diameter/2]),
                        rot=Var(init=[pi/2, 0.0, 0.0])),
    ball2 = Ball,
    joint2 = FreeMotion(obj1=:world, obj2=:(ball2.ball),
                        r=Var(init=[TableX/6+1*distance_balls+dist, 1/2*(diameter+dist), diameter/2]),
                        rot=Var(init=[pi/2, 0.0, 0.0])),
    ball3 = Ball,
    joint3 = FreeMotion(obj1=:world, obj2=:(ball3.ball),
                        r=Var(init=[TableX/6+1*distance_balls+dist, -1/2*(diameter+dist), diameter/2]),
                        rot=Var(init=[pi/2, 0.0, 0.0]))
)

billard = @instantiateModel(buildModia3D(Billard), unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 5.0
testTime = 2.5
tolerance = 1e-8
interval = 0.01
requiredFinalStates = [-0.06367465272629756, 0.24238657622364665, 0.02999809758732255, -0.8055793547583174, 0.5049384414649515, -1.2942520100327887e-6, -41.98758798677127, 1.0302592492754234, -61.383962718912315, 7.327199670112295, 30.778026759406714, -1.8364952739701552, -0.15196276181074325, 0.179157995489763, 0.029997955278049118, -0.5749051726066565, 0.19607773653519803, -9.23765746465685e-7, 1.823582864167988, -0.11082435017660355, 18.026385421294833, -6.535777886518464, 3.361853105628269e-5, 19.163080641737785, 0.415730418200003, -0.25313065448897354, 0.02999753001826395, -0.004216104905417898, -0.49464670233658653, 3.901393071314717e-9, 11.54177678780837, 0.005646322377435987, 0.011097198279087365, 16.486263237425984, -0.25596215953403123, -0.026989059565206447, 0.15504140104701092, -0.18658321851036563, 0.029997948409715088, -0.297314286397064, -0.176662447164565, 4.779910373218372e-7, 0.9427627045555219, 0.6640319309374063, 12.884489822750403, -2.1107480248530672, -5.435105655586106, 9.944019158456165]
simulate!(billard, stopTime=testTime, tolerance=tolerance, interval=interval, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(billard, ["joint0.r" "joint0.rot"; "joint0.v" "joint0.w"], figure=1)
plot(billard, ["joint0.r[3]", "joint0.v[3]"], figure=2)

end
