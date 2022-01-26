module Billard4Balls

using Modia

vmatBalls = VisualMaterial(color="Red", transparency=0.0)
vmatTable = VisualMaterial(color="DarkGreen", transparency=0.5)

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
                                          enableContactDetection=true)),
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
requiredFinalStates = [-0.07460364471173221, 0.2469233096172712, 0.02999808173004779, -0.8140655302596929, 0.5084317411521541, -1.3068678733294896e-6, -42.7652633611384, 1.047560394408595, -60.72880165368908, 7.38708629975639, 31.06670049188193, -1.960629264729011, -0.16442543215744446, 0.18417112667173485, 0.029997936572327664, -0.5830364755926802, 0.19967781792799022, -9.364111798032492e-7, 1.7112859000107352, -0.030080471477740935, 18.428349235400127, -6.655783301501015, 2.608416752777656e-5, 19.434131145447342, 0.4157205359423622, -0.26151248132958926, 0.02999752999578585, -0.004227558410584902, -0.5010189719266817, 4.451418189102991e-9, 11.8212750686071, 0.004051667523321807, 0.009243543858720088, 16.69866719227009, -0.25793872649482263, -0.027881848697937514, 0.14732605173091393, -0.19152425557632252, 0.029997960148676502, -0.3029387840817542, -0.18019916669264857, 4.874004906675558e-7, 0.9559962768568293, 0.514303198441921, 13.15248045495156, -2.245358300329699, -5.500137730579288, 10.136046562118707]
simulate!(billard, stopTime=testTime, tolerance=tolerance, interval=interval, log=true, logStates=false, logEvents=false, logTiming=true, useRecursiveFactorizationUptoSize=500, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(billard, ["joint0.r" "joint0.rot"; "joint0.v" "joint0.w"], figure=1)
#plot(billard, ["joint0.r[1]" "joint0.r[2]" "joint0.r[3]";
#               "joint0.v[1]" "joint0.v[2]" "joint0.v[3]"], figure=2)

end
