module Billard16Balls

using ModiaLang

import Modia3D
using  Modia3D.ModiaInterface

vmatBalls   = Modia3D.VisualMaterial(color="AntiqueWhite", transparency=0.0)
vmatTable   = Modia3D.VisualMaterial(color="DarkGreen", transparency=0.5)
vmatCushion = Modia3D.VisualMaterial(color="Grey20", transparency=0.5)

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
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   enableContactDetection=true,
                                   nominalLength=0.15*TableX,
                                   animationFile="Billard16Balls.json")),
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
requiredFinalStates = [0.9336946243629118, -0.18365520174670474, 0.029998496991356247, -0.7011746198909901, 0.6311295647736829, 1.1287610685608916e-6, -13.537433737049488, -0.3858748206785683, -51.74321001761849, 10.647446137520763, -27.86255858188846, -9.957728295667122, 0.32509152711043904, 0.027780203284739517, 0.029997667930081795, -0.04542743305030694, 0.02500170283086513, 7.062399855287814e-8, 0.9625529100121986, -0.5890320882437217, 1.5160600432975857, -0.8636537095656692, -0.04474632782683813, 1.4954319791715307, 0.39301955928437254, 0.31437311545523294, 0.029997559985557257, -0.0295450598735267, 0.25844118748219774, 6.543232669438406e-8, -7.838021100027451, -0.22760408431777734, 4.052129880389039e-5, -8.613971169221236, -0.015367180725959132, 0.9844568913082455, 0.3526376383583441, -0.14575202949834226, 0.029998721807484585, -0.05704334595170738, -0.09087467006012945, -8.648550230091023e-8, 6.463720232117719, 0.5613406381152121, -0.6619441310051621, 3.0283371525415186, -0.011031122590753485, 1.901031014930388, 0.47669958004172114, 0.30988266338557435, 0.029997421574535003, -0.0006779043504506098, 0.22297266655607906, 2.9010253957924976e-8, -6.65506642882509, -0.004145772945978028, 0.0028226662531805786, -7.4316293203593204, -7.298765565151683e-5, 0.022599728211138887, 0.3783007120971173, -0.012766185812833935, 0.029998759358176016, -0.04494536278891231, -0.018632373536778858, -7.215437557767047e-8, 1.6943216570588908, 0.19414146703780452, 3.298010373792468, -0.3471933000846414, -0.13201026537226748, 1.577730264308742, 0.46331163648978807, -0.09389324485553788, 0.029998896770933176, -0.0016188042316731567, -0.0037739593426910373, -2.5503675848459563e-9, 2.6598529884755653, 0.23804363229547723, 0.3872083559790523, 0.1248900591032485, 0.00033968828158966596, 0.05356980525109636, 0.5432807165029493, 0.2953230757760231, 0.02999730653311251, -0.13624873106020685, -0.18205262515847373, 1.9858018516413702e-7, -5.40642875525656, -0.19653356814243264, 0.09370138552443177, 6.331724950526496, -3.5125692661867474, 2.2385800870275046, 0.5280895349790748, 0.02944513078495663, 0.029997345184240368, -3.671897262472209e-15, -3.6295581814805492e-12, -1.2427691187347466e-15, 1.5884011771883515, 0.00047788925223401856, 0.045046404635795685, 8.53157003717906e-11, -3.8442455469304e-12, 1.2714782913701774e-13, 0.5298393427066304, -0.03176029891751437, 0.029999002794942116, 1.297965371269723e-12, -7.63423453026827e-13, 3.722976278613608e-18, 1.6241627081677217, -0.00011555321024985877, -0.0050541325338022604, 1.7967580704217367e-11, -1.5382972866683138e-12, -3.0499570729309535e-11, 0.5382383412758586, -0.09514048819597622, 0.02999893580898303, 1.3155505360211643e-12, -8.821797446529922e-13, -7.043094683356967e-16, 1.7309170721863256, -0.022807794119219644, -0.28238846622282227, 2.063596477748024e-11, 8.485564273716919e-13, -3.1023757752800775e-11, 0.592498670205686, 0.3681019267809226, 0.029997236420195104, 0.15727976960938742, -0.019558871461817822, -2.644518715680437e-7, -6.163065667728738, 0.37981142806747936, -0.058040028743833724, 0.5349948316934143, 5.243907614414163, -0.34169516938674743, 0.6344294601093406, 0.0669304999587428, 0.029997172347095737, -0.7614223443828724, 0.006309955741959036, 1.2183649383003493e-6, 1.558857816712138, 0.0042029171671443275, -0.7644036324349628, -0.0191002690729473, -0.4380898296432406, 25.377516875983996, 0.7646288540762318, -0.10696914113095653, 0.029998770268321002, 0.15911209364889162, -0.09298081830712687, -2.5596316103907114e-7, 1.9073899574938322, -0.10513813819059165, -6.883833037985919, 3.0989440441527623, 3.5085557038592837e-6, -5.30302146889589, 0.7873220523004107, -0.17884965224773064, 0.029998733802261777, 0.18239453271826178, -0.10527370713656753, -2.9340175118245415e-7, 2.1544182053207317, -0.43508216020191737, -7.5519516937515325, 3.508723323843255, -9.174029482795626e-7, -6.0791204764983995, 0.7976238621443168, -0.24490123678673625, 0.02999869521225786, 0.19303854447667773, -0.11142219870901685, -1.791306576517481e-7, 2.1859628103222946, -0.6263874799512511, -7.873271866785609, 3.7136784821321944, -4.152815543679789e-7, -6.433930426076936]
simulate!(billard, stopTime=testTime, tolerance=tolerance, interval=interval, log=true, logStates=true, logEvents=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(billard, ["joint0.r" "joint0.rot"; "joint0.v" "joint0.w"], figure=1)

end
