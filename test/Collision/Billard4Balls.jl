module Billard4Balls

using Modia3D

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

Billard = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                          enableContactDetection=true)),
    table = Table,
    cushion = Cushion,
    ball0 = Ball,
    joint0 = FreeMotion(obj1=:world, obj2=:(ball0.ball),
                        r=Var(init=Modia.SVector{3,Float64}(-0.8, -0.1, diameter/2)),
                        v=Var(init=Modia.SVector{3,Float64}(3.0, 0.1, 0.0)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball1 = Ball,
    joint1 = FreeMotion(obj1=:world, obj2=:(ball1.ball),
                        r=Var(init=Modia.SVector{3,Float64}(TableX/6, 0.0, diameter/2)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball2 = Ball,
    joint2 = FreeMotion(obj1=:world, obj2=:(ball2.ball),
                        r=Var(init=Modia.SVector{3,Float64}(TableX/6+1*distance_balls+dist, 1/2*(diameter+dist), diameter/2)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball3 = Ball,
    joint3 = FreeMotion(obj1=:world, obj2=:(ball3.ball),
                        r=Var(init=Modia.SVector{3,Float64}(TableX/6+1*distance_balls+dist, -1/2*(diameter+dist), diameter/2)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0)))
)

billard = @instantiateModel(Billard, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 5.0
testTime = 2.5
tolerance = 1e-8
interval = 0.01
requiredFinalStates = [-0.07276225904925236, 0.2453456530297493, 0.029998084681964913, -0.8131729732525697, 0.5077564852782904, -1.3054330162748875e-6, -42.63423883478191, 1.057147543314802, -60.857759531439584, 6.792698034233983, 31.176405211463067, -1.7514957638408275, -0.16326113730274155, 0.18382050752887918, 0.029997938439605354, -0.5825412482752137, 0.19956045698679084, -9.35615015465449e-7, 1.7229244023445067, -0.035543362353461956, 18.391320766547988,
-6.6518712571402085, 2.4718947541234226e-5, 19.417623095449052, 0.41573795678955616, -0.26010343483884174, 0.029997529974735922, -0.004214932154558947, -0.5004735821234532, 4.336935240904981e-9, 11.774329191872447, 0.0041356928296786815, 0.009594145569267762, 16.680464236463397, -0.25872571383738985, -0.03100956782463096, 0.14788438085905078, -0.19122222797434096, 0.029997959251799728, -0.3027500332956257, -0.18012193238323354, 4.870923313573792e-7, 0.9532991179253845, 0.5248074513493877, 13.134794408356132, -2.240587263789671, -5.501727604570828, 10.128445467657514]
simulate!(billard, stopTime=testTime, tolerance=tolerance, interval=interval, log=true, logStates=false, logEvents=false,  useRecursiveFactorizationUptoSize=500, requiredFinalStates=requiredFinalStates) # logTiming=true,

@usingModiaPlot
plot(billard, ["joint0.r" "joint0.rot"; "joint0.v" "joint0.w"], figure=1)
#plot(billard, ["joint0.r[1]" "joint0.r[2]" "joint0.r[3]";
#               "joint0.v[1]" "joint0.v[2]" "joint0.v[3]"], figure=2)

end
