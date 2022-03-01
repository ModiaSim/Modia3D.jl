module Billard16Balls

using Modia3D

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

Billard = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]), mprTolerance = 1.0e-18,
                                   enableContactDetection=true, # maximumContactDamping=1000.0,
                                   nominalLength=0.15*TableX)), # animationFile="Billard16Balls.json"
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
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball4 = Ball,
    joint4 = FreeMotion(obj1=:world, obj2=:(ball4.ball),
                        r=Var(init=Modia.SVector{3,Float64}(TableX/6+2*(distance_balls+dist), (diameter+dist), diameter/2)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball5 = Ball,
    joint5 = FreeMotion(obj1=:world, obj2=:(ball5.ball),
                        r=Var(init=Modia.SVector{3,Float64}(TableX/6+2*(distance_balls+dist), 0.0, diameter/2)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball6 = Ball,
    joint6 = FreeMotion(obj1=:world, obj2=:(ball6.ball),
                        r=Var(init=Modia.SVector{3,Float64}(TableX/6+2*(distance_balls+dist), -(diameter+dist), diameter/2)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball7 = Ball,
    joint7 = FreeMotion(obj1=:world, obj2=:(ball7.ball),
                        r=Var(init=Modia.SVector{3,Float64}(TableX/6+3*(distance_balls+dist), 3/2*(diameter+dist), diameter/2)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball8 = Ball,
    joint8 = FreeMotion(obj1=:world, obj2=:(ball8.ball),
                        r=Var(init=Modia.SVector{3,Float64}(TableX/6+3*(distance_balls+dist), 1/2*(diameter+dist), diameter/2)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball9 = Ball,
    joint9 = FreeMotion(obj1=:world, obj2=:(ball9.ball),
                        r=Var(init=Modia.SVector{3,Float64}(TableX/6+3*(distance_balls+dist), -1/2*(diameter+dist), diameter/2)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball10 = Ball,
    joint10 = FreeMotion(obj1=:world, obj2=:(ball10.ball),
                         r=Var(init=Modia.SVector{3,Float64}(TableX/6+3*(distance_balls+dist), -3/2*(diameter+dist), diameter/2)),
                         rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball11 = Ball,
    joint11 = FreeMotion(obj1=:world, obj2=:(ball11.ball),
                         r=Var(init=Modia.SVector{3,Float64}(TableX/6+4*(distance_balls+dist), 2*(diameter+dist), diameter/2)),
                         rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball12 = Ball,
    joint12 = FreeMotion(obj1=:world, obj2=:(ball12.ball),
                         r=Var(init=Modia.SVector{3,Float64}(TableX/6+4*(distance_balls+dist), (diameter+dist), diameter/2)),
                         rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball13 = Ball,
    joint13 = FreeMotion(obj1=:world, obj2=:(ball13.ball),
                         r=Var(init=Modia.SVector{3,Float64}(TableX/6+4*(distance_balls+dist), 0.0, diameter/2)),
                         rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball14 = Ball,
    joint14 = FreeMotion(obj1=:world, obj2=:(ball14.ball),
                         r=Var(init=Modia.SVector{3,Float64}(TableX/6+4*(distance_balls+dist), -(diameter+dist), diameter/2)),
                         rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),
    ball15 = Ball,
    joint15 = FreeMotion(obj1=:world, obj2=:(ball15.ball),
                         r=Var(init=Modia.SVector{3,Float64}(TableX/6+4*(distance_balls+dist), -2*(diameter+dist), diameter/2)),
                         rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0)))
)

billard = @instantiateModel(Billard, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 5.0
testTime = 1.5
tolerance = 1e-7
interval = 0.01
requiredFinalStates = [0.9332483106072544, -0.18187312532441693, 0.02999849671681763, -0.7095832048149922, 0.6368707654369432, 1.143171041101384e-6, -13.553942563593557, -0.4195243794435807, -51.79233008660278, 11.807007209726729, -27.975810451969565, -9.384295654892883, 0.3251502055431403, 0.02772819926252931, 0.0299976678693217, -0.04538254491852899, 0.024939912003186206, 7.070612097816354e-8, 0.9632341714743309, -0.5877775478177794, 1.5143189193110465, -0.8619057261773254, -0.04563751290039998, 1.493753202547408, 0.3929865232660756, 0.3144313176017014, 0.02999755994676683, -0.029575936325025438, 0.2585011782116537, 6.561905218417997e-8, -7.840063597155187, -0.2277908945852809, -0.00019226199038493234, -8.615970964912231, -0.015375784664400687, 0.9854859077520338, 0.35264619603819136, -0.1455217831653319, 0.029998721964137057, -0.057010908229208075,
-0.09065260935706018, -8.653447595525189e-8, 6.459702521706873, 0.5652888666079168, -0.6639992091264718, 3.020935678454475, -0.010981861879533573, 1.8999486180785712, 0.4767000746505306, 0.3099041129858934, 0.029997421533340347, -0.000677475986573399, 0.22299475711912944, 2.9194977603015673e-8, -6.655776509123009, -0.004144753260705207, 0.0028197921641742747, -7.432365696094535, -7.313664045288635e-5, 0.022585451684508966, 0.37840223311722093, -0.012962335005372607, 0.029998759665832785, -0.04484894718050894, -0.018709076016894303, -7.208853680055674e-8, 1.6964858028612126, 0.1977112296712567, 3.295519051579319, -0.3448993355673044, -0.1360257577294032, 1.575852345040146, 0.4633745589270613, -0.09377988839444025, 0.029998896986993488, -0.0015649341362523825, -0.003652013551363397, -2.4981817856544117e-9, 2.656330472428428, 0.23638530230275412, 0.38603973735519714, 0.1208249566227207, 0.00032866611117734275, 0.051774433544048655, 0.543108869560303, 0.294825435037727, 0.029997306373749883, -0.13739876028170375, -0.18509290020490796, 2.0018394912833252e-7, -5.392323206693009, -0.19893911728985575, 0.09891875505772645, 6.433873311561829,
-3.5319763124007437, 2.2716379105590336, 0.5280319930400308, 0.029427616694857592, 0.029997345415267766, 6.188799580920981e-20, 2.4903463163550593e-21, 4.911639361450669e-16, 1.58895399725716, 0.0005110489112549728, 0.04694833800247477, -6.34969334816791e-20, -2.3228133668370105e-20, -1.4418272517450767e-18, 0.5298438369751591, -0.03183889801132204, 0.029999002925945182, -3.528350416143663e-21, 6.617194627952165e-20, 1.0661056375916704e-15, 1.6267735556275007, -0.00012555443489016495, -0.005204036469135774, -1.5570145480554775e-18, -3.457487981432338e-21, 8.309401741933345e-20, 0.5381964461760903, -0.09511652750222786, 0.029998935973198977, 3.3952835589304447e-19, 4.564806492215736e-20, -1.5820947449642875e-16, 1.7301528615002686, -0.022586636896998424, -0.28101511497365944, -8.512851070761004e-19, -1.56519884944499e-18, -7.861488949920675e-18, 0.5927031648245079, 0.36786146767471367, 0.029997236552983333, 0.15845614558647408, -0.021331046796790255, -2.6683180694948455e-7, -6.1506936540955435, 0.3850250268876852, -0.06208674136021588, 0.5943245171790156, 5.281798549535106, -0.3795624991282996, 0.6246051141249892, 0.06692601485006958,
0.029997185771465026, -0.7827343505535713, 0.0063040893634809525, 1.253509421990429e-6, 1.5554333188869556, -0.0003679414861786754, -0.43663920366531234, -0.012217027572522585, -0.4480160765048617, 26.087850149346753, 0.7644107912334863, -0.10684167972512111, 0.029998770977880018, 0.15888674304279835, -0.09284910325102662, -2.5561782366711e-7, 1.9038478829272165, -0.10274259951709455, -6.876910249442725, 3.0945533634136773, 2.7134425786966793e-6, -5.295509584063961, 0.7873305258304856, -0.17885453451990208, 0.029998734191221205, 0.18240322785924398, -0.10527871347180867, -2.9341817035907206e-7, 2.154470220878549, -0.43523449284704646, -7.552206447838509, 3.5088892765009883, 9.132940300022019e-8, -6.07941081639718, 0.797699578168697, -0.2449449301094816, 0.029998695457177985, 0.19311625283414027, -0.11146703795842461, -1.7920530914190386e-7, 2.1859263471360704, -0.6278127900525268, -7.875763426542622, 3.7151718402931464, 1.9768906588609457e-7, -6.436521510513785]
simulate!(billard, stopTime=testTime, tolerance=tolerance, interval=interval, log=true, useRecursiveFactorizationUptoSize=500, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates) # logTiming=true,

@usingModiaPlot
plot(billard, ["joint0.r" "joint0.rot"; "joint0.v" "joint0.w"], figure=1)
#plot(billard, ["joint0.r[1]" "joint0.r[2]" "joint0.r[3]";
#               "joint0.v[1]" "joint0.v[2]" "joint0.v[3]"], figure=2)
end
