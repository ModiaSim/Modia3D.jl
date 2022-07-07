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

ball = Solid(shape=Sphere(diameter=diameter),
             solidMaterial="BilliardBall",
             visualMaterial=vmatBalls,
             collision=true)

Billard = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   enableContactDetection=true,
                                   mprTolerance=1.0e-18,
#                                  maximumContactDamping=1000.0,
#                                  animationFile="Billard16Balls.json",
                                   nominalLength=0.15*TableX)),
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
                     feature=ball),
    ball4 = Object3D(parent=:world, fixedToParent=false,
                     translation=[TableX/6+2*(distance_balls+dist), (diameter+dist), diameter/2],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=ball),
    ball5 = Object3D(parent=:world, fixedToParent=false,
                     translation=[TableX/6+2*(distance_balls+dist), 0.0, diameter/2],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=ball),
    ball6 = Object3D(parent=:world, fixedToParent=false,
                     translation=[TableX/6+2*(distance_balls+dist), -(diameter+dist), diameter/2],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=ball),
    ball7 = Object3D(parent=:world, fixedToParent=false,
                     translation=[TableX/6+3*(distance_balls+dist), 3/2*(diameter+dist), diameter/2],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=ball),
    ball8 = Object3D(parent=:world, fixedToParent=false,
                     translation=[TableX/6+3*(distance_balls+dist), 1/2*(diameter+dist), diameter/2],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=ball),
    ball9 = Object3D(parent=:world, fixedToParent=false,
                     translation=[TableX/6+3*(distance_balls+dist), -1/2*(diameter+dist), diameter/2],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=ball),
    ball10 = Object3D(parent=:world, fixedToParent=false,
                      translation=[TableX/6+3*(distance_balls+dist), -3/2*(diameter+dist), diameter/2],
                      rotation=[pi/2, 0.0, 0.0],
                      feature=ball),
    ball11 = Object3D(parent=:world, fixedToParent=false,
                      translation=[TableX/6+4*(distance_balls+dist), 2*(diameter+dist), diameter/2],
                      rotation=[pi/2, 0.0, 0.0],
                      feature=ball),
    ball12 = Object3D(parent=:world, fixedToParent=false,
                      translation=[TableX/6+4*(distance_balls+dist), (diameter+dist), diameter/2],
                      rotation=[pi/2, 0.0, 0.0],
                      feature=ball),
    ball13 = Object3D(parent=:world, fixedToParent=false,
                      translation=[TableX/6+4*(distance_balls+dist), 0.0, diameter/2],
                      rotation=[pi/2, 0.0, 0.0],
                      feature=ball),
    ball14 = Object3D(parent=:world, fixedToParent=false,
                      translation=[TableX/6+4*(distance_balls+dist), -(diameter+dist), diameter/2],
                      rotation=[pi/2, 0.0, 0.0],
                      feature=ball),
    ball15 = Object3D(parent=:world, fixedToParent=false,
                      translation=[TableX/6+4*(distance_balls+dist), -2*(diameter+dist), diameter/2],
                      rotation=[pi/2, 0.0, 0.0],
                      feature=ball)
)

billard = @instantiateModel(Billard, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 5.0
testTime = 1.5
tolerance = 1e-7
interval = 0.01
requiredFinalStates = [0.9332444148719927, -0.18186375946917482, 0.029998496723088863, -0.7095844351271156, 0.6368810796715084, 1.1431730539638052e-6, -13.55399177130368, -0.4195497307485026, -51.792329122005135, -21.229276583242317, -23.65271117843232, 7.760065977747092e-9, 0.32515150301575696, 0.02772732166338023, 0.029997667867321085, -0.045381716219676516, 0.02493897962956464, 7.070490674697964e-8, 0.9632401023828058, -0.587752107356861, 1.5142775723756114, -0.8308437907785791, -1.5118949410141513, -3.060402901758649e-10, 0.3929844289831643, 0.314433585122503, 0.029997559950336535, -0.029577874153518823, 0.2585035512496271, 6.562161546067983e-8, -7.840146246994674, -0.22780371618092327, -0.00020162400471457242, -8.616030351812999, -0.9858427871373034, -2.3612472293523234e-14, 0.35264703636149797, -0.14552149720426033, 0.02999872196545602, -0.057010164154041615, -0.09065225367155805, -8.653334203030665e-8, 6.4596790771415185, 0.5652982391883927, -0.6639956770706141, 3.0209829305889535, -1.8998615677380526, -3.721276589601545e-13, 0.47670041160434445, 0.3099020989631557, 0.02999742153252809, -0.0006771745035541265, 0.22299271411893493, 2.919510869411837e-8, -6.655710784224243, -0.004142759835172187, 0.0028186385660788622, -7.4322976108058185, -0.02257007572982667, -2.4790485630984114e-14, 0.3784043982268263, -0.012961976882662853, 0.029998759669299754, -0.04484621513685559, -0.018708473694112566, -7.208415990952376e-8, 1.696489265865553, 0.19770858382045625, 3.2954453850929006, 0.6232505252921184, -1.4939982597406876, -9.35266437450517e-11, 0.4633750773149139, -0.09377922898977689, 0.029998896987810348, -0.0015645848841219327, -0.0036512588493669246, -2.4976340466597957e-9, 2.656310601906443, 0.23637320741757925, 0.38602862990868586, 0.12079995980327145, -0.051763459921091576, -1.461410786250726e-17, 0.5431124217021193, 0.29482608052132353, 0.029997306367926288, -0.1373855246094424, -0.18509962057213744, 2.001605865571896e-7, -5.39236263389228, -0.19889516549128008, 0.09887028605103103, 6.169356127157017, -4.579048975962779, -2.756359955610583e-7, 0.5280322170861772, 0.029427663883163837, 0.02999734541490908, -1.1833800627228365e-19, -8.094474377232591e-21, 7.763826098786717e-17, 1.5889525688422277, 0.0005109396265548114, 0.046941118029524914, 1.8869591285393446e-19, -2.7527864363061138e-18, -2.1533611818933007e-36, 0.529843831576939, -0.03183878092969242, 0.02999900292593586, -5.8525596822436425e-21, -8.850182301368374e-21, 5.424843831620457e-16, 1.6267696790823423, -0.00012554063443316778, -0.005203877134152655, 2.08224161064226e-19, -1.3771645654742136e-19, -2.234919255877932e-36, 0.5381962700678454, -0.0951164266937725, 0.02999893597336097, 1.5087698845429358e-19, 5.912941216301715e-19, 2.1154381102488334e-16, 1.7301497001246133, -0.02258572430603368, -0.28100943887992064, -1.0917573384104563e-17, -7.381005727929696e-18, 2.3864512407351505e-32, 0.5927013302100363, 0.3678637189458646, 0.029997236556235103, 0.15844415347968727, -0.02131857192060813, -2.668097451793402e-7, -6.150776532309457, 0.38497287054741813, -0.06205687194704301, 0.7105058084277128, 5.280630043042688, -7.971472641188731e-9, 0.6246104386396446, 0.0669260214680636, 0.02999718576297974, -0.7827277924590241, 0.006304138587312076, 1.2534990629022721e-6, 1.555434445063634, -0.0003655522498851595, -0.43682627899981, -0.21013560502219034, -26.090634899286027, -1.3792679481687907e-10, 0.7644091593117449, -0.1068407152605166, 0.029998770980499467, 0.15888508724273556, -0.09284811916351207, -2.556151624667579e-7, 1.9038214920984562, -0.1027251696101419, -6.876859354817521, 3.0945196732121394, 5.295454907242669, -1.251811534378311e-12, 0.7873286476747228, -0.17885345102980393, 0.029998734194235585, 0.18240132001945608, -0.10527761167550502, -2.9341510496949545e-7, 2.1544586495625873, -0.4352003315815502, -7.552149739610728, 3.5088518053833653, 6.0793476492161975, 1.0573636616729651e-11, 0.7976924087312456, -0.2449407983426695, 0.02999869546380472, 0.19310891589363471, -0.11146281220155453, -1.791985202250327e-7, 2.1859305170863688, -0.6276776519171341, -7.875527052625196, 3.715030679234973, 6.4362771117007025, 7.752285500022615e-13]
simulate!(billard, stopTime=testTime, tolerance=tolerance, interval=interval, log=true, useRecursiveFactorizationUptoSize=500, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates) # logTiming=true,

@usingModiaPlot
plot(billard, ["ball0.translation" "ball0.rotation"; "ball0.velocity" "ball0.angularVelocity"], figure=1)

end
