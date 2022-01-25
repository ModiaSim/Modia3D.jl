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
                                   enableContactDetection=true, maximumContactDamping=1000.0,
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
interval = 0.001
requiredFinalStates = [0.9333661393766369, -0.18381562888712827, 0.029998497688391915, -0.7014362417219272, 0.6310009997544462, 1.1290899588689932e-6, -13.565404070825176, -0.381058964756716, -51.75885657438144, 10.537345277549955, -27.735368779239312, -10.430136200221227, 0.3250638168669772, 0.027712197685071317, 0.029997668116143905, -0.04558267651330124, 0.024882899734608332, 7.08708375292526e-8, 0.965416714333456, -0.5889389817950592, 1.5175095706521067, -0.8627353305843296, -0.050338947235934135, 1.4988260115560812, 0.3929917006965645, 0.3144655526735176, 0.02999756004737067, -0.02957166616376027, 0.2585362147819237, 6.546959176824667e-8, -7.841172993119799, -0.22772933488389074, -0.00031937205113484224, -8.617138862383795, -0.0153761661581909, 0.9853435747006696, 0.3526558673248896, -0.14526528920171633, 0.029998721894158267, -0.056974608242212996, -0.09040531529599616, -8.644860926138948e-8, 6.455235000096263, 0.5697009425702483, -0.6662857408704342, 3.012693173834483, -0.010923114750204998, 1.8987373661439921, 0.4766983392310379, 0.30995639701896677, 0.029997421593076824, -0.0006790472397150304, 0.2230484330415847, 2.9007914762965347e-8, -6.657481205058328, -0.004158194262967625, 0.0028237265472410952, -7.4341549376124965, -7.354492897781964e-5, 0.0226378054711174, 0.37858459379997833, -0.01303304126802059, 0.029998759898762013, -0.04454851388162656, -0.018598671965046183, -7.152403071858711e-8, 1.7002544724045368, 0.19886559468786247, 3.2892872463916563, -0.3413889150764648, -0.14290695554767294, 1.5650562024658172, 0.46333085370780364, -0.09385847703246966, 0.029998896916633183, -0.0016023154034459151, -0.003736631228246453, -2.524756751505809e-9, 2.658771952706271, 0.2375361890551339, 0.38685341623759373, 0.12364571680165906, 0.00033631987729715405, 0.05302026303612267, 0.5432830285468065, 0.29528415239802475, 0.029997306522725007, -0.1361340886884427, -0.18217533682898493, 1.9837453065516486e-7, -5.405252316032075, -0.19683033148976267, 0.09405739322728604, 6.335836054778727, -3.508750848458166, 2.2362775359298994, 0.5279576840249166, 0.029411604271573133, 0.029997345534074352, -2.0097480807390374e-20, 6.630238551643466e-21, -7.852186594481353e-16, 1.5894787461282525, 0.0005510984167413897, 0.04940034311617822, -1.4726489653714904e-19, 1.6298099217397075e-20, 4.819327329807566e-19, 0.5298457116337745, -0.031876469837529606, 0.029999002929103884, 6.441996653051123e-21, 2.044078647259606e-21, 7.478575748967392e-16, 1.6280210741379746, -0.00013021384141367968, -0.005265784765458889, -6.303145274641481e-20, -9.155654576004988e-21, -1.540158056949745e-19, 0.5382254132148557, -0.09513309283457251, 0.029998935946514784, -2.4353611016104665e-19, 1.1480757656293768e-19, -3.402512648485731e-16, 1.7306808428774456, -0.022739317690155147, -0.28196399908821645, -2.7292648305085957e-18, 1.860055121430653e-19, 5.876967232455233e-18, 0.592522307113131, 0.368128230829464, 0.029997236390732995, 0.15718315723461718, -0.019429213295171666, -2.642803208328939e-7, -6.163666213388699, 0.38062391948289315, -0.05791773996043013, 0.5307694564582713, 5.240733887108378, -0.3393612241165449, 0.6345581404814613, 0.06693110909072808, 0.029997172271377055, -0.7610317598699818, 0.006309980725063319, 1.2177417150428154e-6, 1.5589113127222334, 0.004251050614008636, -0.7683880059515366, -0.019236388490959964, -0.43789479945224935, 25.364498817249434, 0.7644490357187969, -0.10686411031544148, 0.029998770706948247, 0.15892605579721072, -0.09287215208524814, -2.5564504515598876e-7, 1.904457619305972, -0.1031523714772522, -6.878100110409454, 3.095322174103587, 3.4875825666557394e-6, -5.296819787141439, 0.7873764400470138, -0.17888103685815732, 0.029998733869831, 0.18245039270778482, -0.10530593919198675, -2.934692365195682e-7, 2.154753570394882, -0.4360610751585018, -7.553586183876722, 3.5097973316114093, -1.098206217365888e-6, -6.0809827701210315, 0.7977083789714665, -0.24495001552925505, 0.02999869529308616, 0.19312533440021248, -0.11147228664501829, -1.7918625043209475e-7, 2.1859222338974504, -0.6279787289271236, -7.876052995028591, 3.7153480112389947, -1.3327607189387022e-6, -6.436823566539937]
simulate!(billard, stopTime=testTime, tolerance=tolerance, interval=interval, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(billard, ["joint0.r" "joint0.rot"; "joint0.v" "joint0.w"], figure=1)

end
