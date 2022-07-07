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

ball = Solid(shape=Sphere(diameter=diameter),
                          solidMaterial="BilliardBall",
                          visualMaterial=vmatBalls,
                          collision=true)

Billard = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                          enableContactDetection=true)),
    table = Table,
    cushion = Cushion,
    ball0 = Object3D(parent=:world, fixedToParent=false,
                     translation=[-0.8, -0.1, diameter/2],
                     velocity=[3.0, 0.1, 0.0],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=ball),
    ball1 = Object3D(parent=:world, fixedToParent=false,
                     translation=[TableX/6, 0.0, diameter/2],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=ball),
    ball2 = Object3D(parent=:world, fixedToParent=false,
                     translation=[TableX/6+1*distance_balls+dist, 1/2*(diameter+dist), diameter/2],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=ball),
    ball3 = Object3D(parent=:world, fixedToParent=false,
                     translation=[TableX/6+1*distance_balls+dist, -1/2*(diameter+dist), diameter/2],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=ball)
)

billard = @instantiateModel(Billard, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 5.0
testTime = 2.5
tolerance = 1e-8
interval = 0.01
requiredFinalStates = [-0.07275931054522237, 0.245345929634687, 0.029998084686692236, -0.8131715869137905, 0.5077564513202036, -1.3054307952123734e-6, -42.63401141766014, 1.0572321974473446, -60.85792815772321, -16.92514249042554, -27.10560336928497, -2.0098190928760517e-7, -0.16326306297598359, 0.1838211724164155, 0.02999793843651809, -0.5825422921046868, 0.1995608242777997, -9.356166915346368e-7, 1.7229051326940281, -0.03553401956048332, 18.391382536425706, -6.651884042852161, -19.41765770491842, -1.5008949570945262e-10, 0.41573792401066556, -0.26010411034360825, 0.029997529974785185, -0.0042149501111542675, -0.5004738043885597, 4.337011224944419e-9, 11.774352676190583, 0.004135604875985572, 0.009594011344653352, 16.681915255777486, -0.14049374801446188, 9.991754111353087e-13, 0.14788382539760303, -0.1912228312019073, 0.029997959252692587, -0.3027500651996917, -0.18012221710117013, 4.87092389687382e-7, 0.9533014101224873, 0.524794916392587, 13.134822395629266, 6.003727678413058, -10.091086903842799, -9.90430599231809e-12]
simulate!(billard, stopTime=testTime, tolerance=tolerance, interval=interval, log=true, logStates=false, logEvents=false,  useRecursiveFactorizationUptoSize=500, requiredFinalStates=requiredFinalStates) # logTiming=true,

@usingPlotPackage
plot(billard, ["ball0.translation" "ball0.rotation"; "ball0.velocity" "ball0.angularVelocity"], figure=1)

end
