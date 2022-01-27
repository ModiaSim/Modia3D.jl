module Billard16Balls

using Modia

vmatBalls   = VisualMaterial(color="AntiqueWhite", transparency=0.0)
vmatTable   = VisualMaterial(color="DarkGreen", transparency=0.5)
vmatCushion = VisualMaterial(color="Grey20", transparency=0.5)

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
                                       visualMaterial=vmatCushion)),
    cushion12 = Object3D(parent=:cushion11,
                         translation=[-LyBox/4, 0.0, hz/2+hzz/2],
                         feature=Solid(shape=Box(lengthX=1.5*LyBox, lengthY=TableY, lengthZ=hzz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatCushion,
                                       collision=true)),
    cushion31 = Object3D(parent=:world,
                         translation=[-TableX/2+LyBox/2, 0.0, hz/2],
                         feature=Solid(shape=Box(lengthX=LyBox, lengthY=TableY, lengthZ=hz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatCushion)),
    cushion32 = Object3D(parent=:cushion31,
                         translation=[LyBox/4, 0.0, hz/2+hzz/2],
                         feature=Solid(shape=Box(lengthX=1.5*LyBox, lengthY=TableY, lengthZ=hzz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatCushion,
                                       collision=true)),
    cushion21 = Object3D(parent=:world,
                         translation=[0.0, TableY/2-LyBox/2, hz/2],
                         feature=Solid(shape=Box(lengthX=TableX, lengthY=LyBox, lengthZ=hz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatCushion)),
    cushion22 = Object3D(parent=:cushion21,
                         translation=[0.0, -LyBox/4, hz/2+hzz/2],
                         feature=Solid(shape=Box(lengthX=TableX, lengthY=1.5*LyBox, lengthZ=hzz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatCushion,
                                       collision=true)),
    cushion41 = Object3D(parent=:world,
                         translation=[0.0, -TableY/2+LyBox/2, hz/2],
                         feature=Solid(shape=Box(lengthX=TableX, lengthY=LyBox, lengthZ=hz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatCushion)),
    cushion42 = Object3D(parent=:cushion41,
                         translation=[0.0, LyBox/4, hz/2+hzz/2],
                         feature=Solid(shape=Box(lengthX=TableX, lengthY=1.5*LyBox, lengthZ=hzz),
                                       solidMaterial="BilliardCushion",
                                       visualMaterial=vmatCushion,
                                       collision=true))
)

Ball = Model(
    ball = Object3D(feature=Solid(shape=Sphere(diameter=diameter),
                                  solidMaterial="BilliardBall",
                                  visualMaterial=vmatBalls,
                                  collision=true))
)

Billard = Model(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]), mprTolerance = 1.0e-18,
                                   enableContactDetection=true, # maximumContactDamping=1000.0,
                                   nominalLength=0.15*TableX)), # animationFile="Billard16Balls.json"
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
                        rot=Var(init=[pi/2, 0.0, 0.0])),
    ball4 = Ball,
    joint4 = FreeMotion(obj1=:world, obj2=:(ball4.ball),
                        r=Var(init=[TableX/6+2*(distance_balls+dist), (diameter+dist), diameter/2]),
                        rot=Var(init=[pi/2, 0.0, 0.0])),
    ball5 = Ball,
    joint5 = FreeMotion(obj1=:world, obj2=:(ball5.ball),
                        r=Var(init=[TableX/6+2*(distance_balls+dist), 0.0, diameter/2]),
                        rot=Var(init=[pi/2, 0.0, 0.0])),
    ball6 = Ball,
    joint6 = FreeMotion(obj1=:world, obj2=:(ball6.ball),
                        r=Var(init=[TableX/6+2*(distance_balls+dist), -(diameter+dist), diameter/2]),
                        rot=Var(init=[pi/2, 0.0, 0.0])),
    ball7 = Ball,
    joint7 = FreeMotion(obj1=:world, obj2=:(ball7.ball),
                        r=Var(init=[TableX/6+3*(distance_balls+dist), 3/2*(diameter+dist), diameter/2]),
                        rot=Var(init=[pi/2, 0.0, 0.0])),
    ball8 = Ball,
    joint8 = FreeMotion(obj1=:world, obj2=:(ball8.ball),
                        r=Var(init=[TableX/6+3*(distance_balls+dist), 1/2*(diameter+dist), diameter/2]),
                        rot=Var(init=[pi/2, 0.0, 0.0])),
    ball9 = Ball,
    joint9 = FreeMotion(obj1=:world, obj2=:(ball9.ball),
                        r=Var(init=[TableX/6+3*(distance_balls+dist), -1/2*(diameter+dist), diameter/2]),
                        rot=Var(init=[pi/2, 0.0, 0.0])),
    ball10 = Ball,
    joint10 = FreeMotion(obj1=:world, obj2=:(ball10.ball),
                         r=Var(init=[TableX/6+3*(distance_balls+dist), -3/2*(diameter+dist), diameter/2]),
                         rot=Var(init=[pi/2, 0.0, 0.0])),
    ball11 = Ball,
    joint11 = FreeMotion(obj1=:world, obj2=:(ball11.ball),
                         r=Var(init=[TableX/6+4*(distance_balls+dist), 2*(diameter+dist), diameter/2]),
                         rot=Var(init=[pi/2, 0.0, 0.0])),
    ball12 = Ball,
    joint12 = FreeMotion(obj1=:world, obj2=:(ball12.ball),
                         r=Var(init=[TableX/6+4*(distance_balls+dist), (diameter+dist), diameter/2]),
                         rot=Var(init=[pi/2, 0.0, 0.0])),
    ball13 = Ball,
    joint13 = FreeMotion(obj1=:world, obj2=:(ball13.ball),
                         r=Var(init=[TableX/6+4*(distance_balls+dist), 0.0, diameter/2]),
                         rot=Var(init=[pi/2, 0.0, 0.0])),
    ball14 = Ball,
    joint14 = FreeMotion(obj1=:world, obj2=:(ball14.ball),
                         r=Var(init=[TableX/6+4*(distance_balls+dist), -(diameter+dist), diameter/2]),
                         rot=Var(init=[pi/2, 0.0, 0.0])),
    ball15 = Ball,
    joint15 = FreeMotion(obj1=:world, obj2=:(ball15.ball),
                         r=Var(init=[TableX/6+4*(distance_balls+dist), -2*(diameter+dist), diameter/2]),
                         rot=Var(init=[pi/2, 0.0, 0.0]))
)

billard = @instantiateModel(buildModia3D(Billard), unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 5.0
testTime = 1.5
tolerance = 1e-7
interval = 0.01
requiredFinalStates = [0.932366799335576, -0.1809055453453808, 0.029998498134144663, -0.7103630227248175, 0.6377388248964195, 1.1444288897717778e-6, -13.566618270564867, -0.42919381686392427, -51.78988526736147, 11.493760035228535, -28.18604746494348, -9.274500717106385, 0.3251486817061108, 0.0277286799792369, 0.029997667871541817, -0.045386147393685815, 0.02494087446146572, 7.071173322231522e-8, 0.9632466512796863, -0.5878042027751957, 1.5143706927695095, -0.861956899427833, -0.045669513590901455, 1.493862089438712, 0.39298296990029624, 0.31443841595786626, 0.02999755995288625, -0.029579270256977687, 0.25850852424613885, 6.562293682475234e-8, -7.840310943130716, -0.22781027762967296, -0.00022046380069404116, -8.61621588049777, -0.015376628222091354, 0.9855966966155278, 0.3526455762477307, -0.14552368823544187, 0.029998721963160897, -0.057011659723902604, -0.09065452423116344, -8.653552121260222e-8, 6.459748235199671, 0.5652530149121028, -0.6639862633740976, 3.0209995132661214, -0.010982860555903253, 1.8999736576736441, 0.47670063132250745, 0.3099048889509481, 0.029997421532352294, -0.0006769792995982574, 0.2229955863299179, 2.9194050230091563e-8, -6.655801757380954, -0.004141770772412782, 0.0028176900827465165, -7.4323933372165625, -7.305657972955619e-5, 0.022568896128615564, 0.37840319713002285, -0.012963232464096818, 0.02999875966729973, -0.04484612026412316, -0.01870915329881244, -7.208399687395234e-8, 1.6965213695410617, 0.1977224261883109, 3.2954869765056243, -0.34490572158434835, -0.13607654959318882, 1.5757582373497208, 0.4633742499427428, -0.09378070068856673, 0.029998896986446838, -0.0015652873740667117, -0.003652848226545061, -2.4987276746535646e-9, 2.656356306554525, 0.23639487319578706, 0.38604413792469905, 0.12085277944201457, 0.0003287441742882174, 0.05178620920634418, 0.5428863683537681, 0.2943322001786872, 0.029997306661373246, -0.13768336551274443, -0.1851622791468273, 2.0078211428767719e-7, -5.375047117690531, -0.20203914599329129, 0.10653255747034371, 6.433295956341206, -3.5272618482549762, 2.3057689265206, 0.5280309277659228, 0.02942739270768953, 0.02999734541680833, -5.044121808188397e-20, 9.428347202819351e-21, -5.466235560496825e-16, 1.5889609635189603, 0.0005115878793856781, 0.046982911615666534, -2.4228143760058907e-19, 3.2624724384601617e-20, 1.1673908074254789e-18, 0.5298438682763498, -0.03183946031101317, 0.029999002925936664, -8.142477014259519e-21, 1.1761024407502177e-19, 1.0608751087429057e-15, 1.6267922291325265, -0.0001256272854985119, -0.005205072094840777, -2.7673423988914685e-18, -3.6811347353528955e-21, 1.9164480962300681e-19, 0.5381967274955715, -0.09511668839776263, 0.02999893597287435, 1.3871304758602952e-19, 7.498953839924982e-20, -8.143802396256386e-16, 1.730158036566943, -0.022588130601776637, -0.281024412874448, -1.5665221939228563e-18, -9.961009508542885e-19, -3.2119014261001362e-18, 0.5929249250619509, 0.3675391345013976, 0.02999723614557378, 0.15871532013868042, -0.02163412259019427, -2.6738370458370294e-7, -6.138533187062709, 0.39137705412259355, -0.06719579923842542, 0.6037384255409345, 5.286894168535472, -0.4297978224995873, 0.6237332809243649, 0.06693523635562337, 0.02999718716466229, -0.7831165950290768, 0.006322740845781519, 1.2540953236069932e-6, 1.5553640384963576, -0.0007184754118179108, -0.4076245039066669, -0.01658144207454254, -0.44593094784613335, 26.100631753797174, 0.7644111449675985, -0.10684189228601401, 0.029998770977235967, 0.15888713100972973, -0.09284933646323527, -2.556184630160367e-7, 1.9038535203035778, -0.10274628247224886, -6.876920865937453, 3.0945607771911487, 3.0099334930590755e-6, -5.295522727127085, 0.7873309564302046, -0.17885480694204717, 0.02999873419045156, 0.18240369275975934, -0.10527901745690532, -2.9341893832220583e-7, 2.1544733025511604, -0.43524252418696596, -7.552219570239919, 3.508900484679646, 4.884908275674251e-8, -6.079425693001544, 0.7976927318357205, -0.2449410071153632, 0.029998695463387254, 0.19310925173414872, -0.11146304026743674, -1.7919890060996748e-7, 2.1859301340330397, -0.6276845534744078, -7.87553854587797, 3.7150403761585165, 4.196632621604925e-7, -6.436287097720782]
simulate!(billard, stopTime=testTime, tolerance=tolerance, interval=interval, log=true, useRecursiveFactorizationUptoSize=500, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates) # logTiming=true,

@usingModiaPlot
plot(billard, ["joint0.r" "joint0.rot"; "joint0.v" "joint0.w"], figure=1)
#plot(billard, ["joint0.r[1]" "joint0.r[2]" "joint0.r[3]";
#               "joint0.v[1]" "joint0.v[2]" "joint0.v[3]"], figure=2)
end
