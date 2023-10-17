module YouBotPingPong

using Modia3D

include("$(Modia3D.modelsPath)/Blocks.jl")
include("$(Modia3D.modelsPath)/Electric.jl")
include("$(Modia3D.modelsPath)/Rotational.jl")

# some constants

# visual materials
vmat1 = VisualMaterial(color="LightBlue"  , transparency=0.0)  # material of FileMesh
vmat2 = VisualMaterial(color="DarkGrey"   , transparency=0.0)  # material of work piece
vmat3 = VisualMaterial(color="DodgerBlue3", transparency=0.0)  # material of table
vmat4 = VisualMaterial(color=[255,215,0]  , transparency=0.0)  # material of ground

xPosTable = 0.71
tableX    = 0.3
tableY    = 0.3
tableZ    = 0.01
heigthLeg = 0.37
widthLeg  = 0.02
diameter = 0.05
right_finger_offset=0.0

# all needed paths to fileMeshes
base_frame_obj           = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/base_frame.obj")
plate_obj                = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/plate.obj")
front_right_wheel_obj    = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/front-right_wheel.obj")
front_left_wheel_obj     = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/front-left_wheel.obj")
back_right_wheel_obj     = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/back-right_wheel.obj")
back_left_wheel_obj      = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/back-left_wheel.obj")
arm_base_frame_obj       = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_base_frame.obj")
arm_joint_1_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_1.obj")
arm_joint_2_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_2.obj")
arm_joint_3_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_3.obj")
arm_joint_4_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_4.obj")
arm_joint_5_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_5.obj")
gripper_base_frame_obj   = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/gripper_base_frame.obj")
gripper_left_finger_obj  = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/gripper_left_finger.obj")
gripper_right_finger_obj = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/gripper_right_finger.obj")

# Drive train data: motor inertias and gear ratios
nullRot = [0, 0, 0]

motorInertia1 = 0.0000135 + 0.000000409
motorInertia2 = 0.0000135 + 0.000000409
motorInertia3 = 0.0000135 + 0.000000071
motorInertia4 = 0.00000925 + 0.000000071
motorInertia5 = 0.0000035 + 0.000000071
gearRatio1    = 156.0
gearRatio2    = 156.0
gearRatio3    = 100.0
gearRatio4    = 71.0
gearRatio5    = 71.0

m1=1.390
translation1 =[0.033,0,0]
rotation1 = [180u"°", 0, 0]

m2=1.318
translation2=[0.155,0,0]
rotation2 = [90u"°", 0.0, -90u"°"]

m3=0.821
translation3=[0,0.135,0]
rotation3 = [0, 0, -90u"°"]

m4=0.769
translation4=[0,0.11316,0]

m5=0.687
translation5=[0,0,0.05716]
rotation5 = [-90u"°", 0, 0]

### ----------------- Servo Model -----------------------
# parameters for Link
axisLink = 3
dLink  = 1.0
k1Link = 50.0
k2Link = 0.1
T2Link = 1.0
# parameters for Gripper
axisGripper = 2
dGripper  = 1.0
k1Gripper = 50.0
k2Gripper = 0.1
T2Gripper = 1.0
motorInertiaGripper = 0.0
gearRatioGripper    = 1.0

#### ----------- Robot Program ------------------
initPosition = [0.0,     0.0,     pi/2,   0.0,    0.0,   0.0]

function robotProgram(robotActions)
    addReferencePath(robotActions,
    names = ["angle1", "angle2", "angle3", "angle4", "angle5", "gripper"],
    position = initPosition,
    v_max =    [2.68512, 2.68512, 4.8879, 5.8997, 5.8997, 1.0],
    a_max =    [1.5, 1.5, 1.5, 1.5, 1.5, 0.5])

    ptpJointSpace(robotActions,
    [0.0 0.0 pi/2     0.0 0.0 0.0;
    0.0  0.3 pi/2-0.3 0.0 0.0 0.0;
    0.0  0.0 0.0      0.0 0.0 0.0])

    return nothing
end

# Controller Model
Controller = Model(
    # Interface
    refLoadAngle     = input,
    actualMotorAngle = input,
    actualMotorSpeed = input,
    refTorque        = output,
    # Components
    gainOuter     = Gain,
    angleFeedback = Feedback,
    gainInner     = Gain,
    speedFeedback = Feedback,
    PI = PI,

    connect = :[
        (refLoadAngle, gainOuter.u)
        (gainOuter.y, angleFeedback.u1)
        (actualMotorAngle, angleFeedback.u2)
        (angleFeedback.y, gainInner.u)
        (gainInner.y, speedFeedback.u1)
        (actualMotorSpeed, speedFeedback.u2)
        (speedFeedback.y, PI.u)
        (PI.y, refTorque)]
)

Drive = Model(
    # Interface
    refTorque   = input,
    actualSpeed = output,
    actualAngle = output,
    flange      = Flange,
    # Components
    torque       = UnitlessTorque,
    speedSensor  = UnitlessSpeedSensor,
    angleSensor  = UnitlessAngleSensor,
    motorInertia = Inertia,
    idealGear    = IdealGear,

    connect = :[
        (refTorque, torque.tau)
        (torque.flange, speedSensor.flange, angleSensor.flange, motorInertia.flange_a)
        (motorInertia.flange_b, idealGear.flange_a)
        (idealGear.flange_b, flange)
        (speedSensor.w, actualSpeed)
        (angleSensor.phi, actualAngle)
        ]
)

Servo = Model(
    refLoadAngle = input,
    flange       = Flange,
    ppi          = Controller,
    drive        = Drive,

    connect = :[
        # inputs of ppi
        (refLoadAngle, ppi.refLoadAngle)
        (ppi.actualMotorSpeed, drive.actualSpeed)
        (ppi.actualMotorAngle, drive.actualAngle)
        # output of ppi --> input of motor
        (ppi.refTorque, drive.refTorque)
        # Flange of drive
        (drive.flange, flange)
        ]
)

Ground = Model(
    ground = Object3D(parent=:world,
        translation=[xPosTable, 0.0, -0.005], feature = Solid(shape =Box(lengthX=2.5, lengthY=2.5, lengthZ=0.01), collisionSmoothingRadius=0.001, solidMaterial="DryWood", visualMaterial=vmat4, collision=true))
)

Table = Model(
    plate = Object3D(parent=:world, translation=[xPosTable, 0.0, heigthLeg],
        feature=Solid(shape=Box(lengthX=tableX, lengthY=tableY, lengthZ=tableZ),
        collisionSmoothingRadius=0.001, solidMaterial="BilliardTable", collision=true, visualMaterial=vmat3)),
    leg1  = Object3D(parent=:plate, translation=[ tableX/2-widthLeg/2,  tableY/2-widthLeg/2, -heigthLeg/2],
        feature=Solid(shape=Box(lengthX=widthLeg, lengthY=widthLeg, lengthZ=heigthLeg),
        solidMaterial="DryWood", visualMaterial=vmat3)),
    leg2  = Object3D(parent=:plate, translation=[-tableX/2+widthLeg/2,  tableY/2-widthLeg/2, -heigthLeg/2],
        feature=Solid(shape=Box(lengthX=widthLeg, lengthY=widthLeg, lengthZ=heigthLeg), solidMaterial="DryWood", visualMaterial=vmat3)),
    leg3  = Object3D(parent=:plate, translation=[ tableX/2-widthLeg/2, -tableY/2+widthLeg/2, -heigthLeg/2],
        feature=Solid(shape=Box(lengthX=widthLeg, lengthY=widthLeg, lengthZ=heigthLeg), solidMaterial="DryWood", visualMaterial=vmat3)),
    leg4  = Object3D(parent=:plate, translation=[-tableX/2+widthLeg/2, -tableY/2+widthLeg/2, -heigthLeg/2],
        feature=Solid(shape=Box(lengthX=widthLeg, lengthY=widthLeg, lengthZ=heigthLeg), solidMaterial="DryWood", visualMaterial=vmat3))
)

Base = Model(
    rota = Par(value = :(nullRot)),
    trans = [0.0,0.0,0.0],
    base_frame = Object3D(parent=:world, rotation = :rota,
        translation=:(trans + [0, 0, 0.084]),
        feature=Solid(shape=FileMesh(filename = base_frame_obj), massProperties=MassPropertiesFromShapeAndMass(mass=19.803), visualMaterial=vmat1 )),
    plate = Object3D(parent=:base_frame, rotation = :rota,
        translation=[-0.159, 0, 0.046],
        feature=Solid(shape=FileMesh(filename = plate_obj), massProperties=MassPropertiesFromShapeAndMass(mass=2.397), visualMaterial=vmat1)),
    front_right_wheel = Object3D(parent=:base_frame, rotation = :rota,
        translation=[0.228, -0.158, -0.034],
        feature=Solid(shape=FileMesh(filename = front_right_wheel_obj), massProperties=MassPropertiesFromShapeAndMass(mass=1.4), visualMaterial=vmat1)),
    front_left_wheel = Object3D(parent=:base_frame, rotation = :rota,
        translation=[0.228, 0.158, -0.034],
        feature=Solid(shape=FileMesh(filename = front_left_wheel_obj), massProperties=MassPropertiesFromShapeAndMass(mass=1.4), visualMaterial=vmat1)),
    back_right_wheel = Object3D(parent=:base_frame, rotation = :rota,
        translation=[-0.228, -0.158, -0.034],
        feature=Solid(shape=FileMesh(filename = back_right_wheel_obj), massProperties=MassPropertiesFromShapeAndMass(mass=1.4), visualMaterial=vmat1)),
    back_left_wheel   = Object3D(parent=:base_frame, rotation = :rota,
        translation=[-0.228, 0.158, -0.034],
        feature=Solid(shape=FileMesh(filename = back_left_wheel_obj), massProperties=MassPropertiesFromShapeAndMass(mass=1.4), visualMaterial=vmat1))
)

servoParameters1 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio1),
                          gainInner = Map(k=k1Link),
                          PI        = Map(k=k2Link, T=T2Link)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia1),
                          idealGear = Map(ratio=gearRatio1)
                      )
                  )

servoParameters2 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio2),
                          gainInner = Map(k=k1Link),
                          PI        = Map(k=k2Link, T=T2Link)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia2),
                          idealGear = Map(ratio=gearRatio2)
                      )
                  )

servoParameters3 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio3),
                          gainInner = Map(k=k1Link),
                          PI        = Map(k=k2Link, T=T2Link)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia3),
                          idealGear = Map(ratio=gearRatio3)
                      )
                  )

servoParameters4 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio4),
                          gainInner = Map(k=k1Link),
                          PI        = Map(k=k2Link, T=T2Link)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia4),
                          idealGear = Map(ratio=gearRatio4)
                      )
                  )

servoParameters5 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio5),
                          gainInner = Map(k=k1Link),
                          PI        = Map(k=k2Link, T=T2Link)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia5),
                          idealGear = Map(ratio=gearRatio5)
                      )
                  )

featureBody1 = Solid(shape = FileMesh(filename = arm_joint_1_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m1), visualMaterial=vmat1)
featureBody2 = Solid(shape = FileMesh(filename = arm_joint_2_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m2), visualMaterial=vmat1)
featureBody3 = Solid(shape = FileMesh(filename = arm_joint_3_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m3), visualMaterial=vmat1)
featureBody4 = Solid(shape = FileMesh(filename = arm_joint_4_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m4), visualMaterial=vmat1)
featureBody5 = Solid(shape = FileMesh(filename = arm_joint_5_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m5), visualMaterial=vmat1)

linkParameters1 = Map(
                    parent1 = Par(value = :(armBase_b)),
                    featureBody = featureBody1, # Par(value = :featureBody1),
                    initRefPos = initPosition[1],
                    trans = translation1,
                    rota = Par(value = :(rotation1))
)

linkParameters2 = Map(
                    parent1 = Par(value = :(link1.obj2)),
                    featureBody = featureBody2, #Par(value = :featureBody2),
                    m = m2,
                    initRefPos = initPosition[2],
                    trans = translation2,
                    rota = Par(value = :(rotation2))
)

linkParameters3 = Map(
                    parent1 = Par(value = :(link2.obj2)),
                    featureBody = featureBody3, # Par(value = :featureBody3),
                    m = m3,
                    initRefPos = initPosition[3],
                    trans = translation3,
                    rota = Par(value = :(rotation3))
)

linkParameters4 = Map(
                    parent1 = Par(value = :(link3.obj2)),
                    featureBody = featureBody4, # Par(value = :featureBody4),
                    m = m4,
                    initRefPos = initPosition[4],
                    trans = translation4,
                    rota = Par(value = :(nullRot))
)

linkParameters5 = Map(
                    parent1 = Par(value = :(link4.obj2)),
                    featureBody = featureBody5, # Par(value = :featureBody5),
                    m = m5,
                    initRefPos = initPosition[5],
                    trans = translation5,
                    rota = Par(value = :(rotation5))
)

featureVisual1 = Visual(shape=Cylinder(axis=3, diameter=0.01, length=0.12), visualMaterial=vmat1)

Link = Model(
    parent1 = Par(value = :(nothing)),
    featureBody = featureBody1,
    # featureVisual = featureVisual1,
    trans = [0,0,0],
    rota = Par(value = :(nullRot)),

    obj1 = Object3D(parent=:parent1, rotation=:rota), # feature=:featureVisual,
    body = Object3D(feature = :featureBody),
    obj2 = Object3D(parent =:body, translation = :trans),
)

Gripper = Model(
    obj1 = Object3D(parent=:(link5.obj2), rotation=[0, 0, -180u"°"]),
    gripper_base_frame = Object3D(parent=:obj1, feature=Solid(shape=
        FileMesh(filename = gripper_base_frame_obj), massProperties=MassPropertiesFromShapeAndMass(mass=0.199), visualMaterial=vmat1)),
    gripper_left_finger_a = Object3D(parent=:gripper_base_frame),
    gripper_left_finger = Object3D(parent=:gripper_left_finger_a,
        translation=[0, 0.0082, 0], feature=Solid(shape= FileMesh(filename = gripper_left_finger_obj), massProperties=MassPropertiesFromShapeAndMass(mass=0.010), visualMaterial=vmat1, contactMaterial="BilliardCue", collision=true)),
    gripper_right_finger_a = Object3D(parent=:gripper_base_frame,
        translation=[0,-right_finger_offset,0]),
    gripper_right_finger = Object3D(parent=:gripper_right_finger_a, translation=[0, -0.0082, 0],
        feature = Solid(shape = FileMesh(filename = gripper_right_finger_obj), massProperties=MassPropertiesFromShapeAndMass(mass=0.010), visualMaterial=vmat1, contactMaterial="BilliardCue", collision=true))
)

YouBot(worldName) = Model(
    base = Base,
    worldName = Par(worldName),
    arm_base_frame = Object3D(parent=:(base.base_frame),
        translation=[0.143, 0.0, 0.046],
        feature = Solid(shape = FileMesh(filename = arm_base_frame_obj), massProperties=MassPropertiesFromShapeAndMass(mass=0.961), visualMaterial=vmat1)),
    armBase_b = Object3D(parent=:arm_base_frame, translation=[0.024, 0, 0.115]),

    link1 = Link | linkParameters1,
    link2 = Link | linkParameters2,
    link3 = Link | linkParameters3,
    link4 = Link | linkParameters4,
    link5 = Link | linkParameters5,
    gripper = Gripper,

    rev1 = RevoluteWithFlange(obj1 = :(link1.obj1), obj2 = :(link1.body),
        axis=axisLink, phi = Var(init = initPosition[1]), w=Var(init=0.0)),
    rev2 = RevoluteWithFlange(obj1 = :(link2.obj1), obj2 = :(link2.body),
        axis=axisLink, phi = Var(init = initPosition[1]), w=Var(init=0.0)),
    rev3 = RevoluteWithFlange(obj1 = :(link3.obj1), obj2 = :(link3.body),
        axis=axisLink, phi = Var(init = initPosition[1]), w=Var(init=0.0)),
    rev4 = RevoluteWithFlange(obj1 = :(link4.obj1), obj2 = :(link4.body),
        axis=axisLink, phi = Var(init = initPosition[1]), w=Var(init=0.0)),
    rev5 = RevoluteWithFlange(obj1 = :(link5.obj1), obj2 = :(link5.body),
        axis=axisLink, phi = Var(init = initPosition[1]), w=Var(init=0.0)),

    servo1 = Servo,
    servo2 = Servo,
    servo3 = Servo,
    servo4 = Servo,
    servo5 = Servo,

    modelActions = ModelActions(world=:world, actions=robotProgram),
    currentAction = Var(hideResult=true),

    equations=:[
        currentAction = executeActions(modelActions),
        servo1.refLoadAngle = getRefPathPosition(currentAction, 1),
        servo2.refLoadAngle = getRefPathPosition(currentAction, 2),
        servo3.refLoadAngle = getRefPathPosition(currentAction, 3),
        servo4.refLoadAngle = getRefPathPosition(currentAction, 4),
        servo5.refLoadAngle = getRefPathPosition(currentAction, 5)
    ],

    connect = :[
        (servo1.flange, rev1.flange)
        (servo2.flange, rev2.flange)
        (servo3.flange, rev3.flange)
        (servo4.flange, rev4.flange)
        (servo5.flange, rev5.flange)
        ]
)

animationFile = nothing  # "YouBotPingPong.json"

Setup = Model3D(
    gravField = UniformGravityField(g=9.81, n=[0,0,-1]),
    world = Object3D(feature=Scene(gravityField=:gravField, visualizeFrames=false, nominalLength=3*tableX, defaultFrameLength=0.1, enableContactDetection=true, elasticContactReductionFactor=1e-4, animationFile=animationFile)),

    # worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length = 0.5))),

    table = Table,
    ground = Ground,

    sphere1 = Object3D(parent=:(table.plate), fixedToParent=false,
                       translation=[-tableX/2+diameter/2+0.001, 0.0002, diameter/2+tableZ/2],
                       rotation=[pi/2, 0.0, 0.0],
                       feature=Solid(shape=Sphere(diameter=diameter),
                                     visualMaterial=vmat2,
                                     solidMaterial="BilliardBall",
                                     collision=true)),

    sphere2 = Object3D(parent=:(table.plate), fixedToParent=false,
                       translation=[tableX/2-diameter/2-0.0001, -0.003, diameter/2+tableZ/2],
                       rotation=[pi/2, 0.0, 0.0],
                       feature=Solid(shape=Sphere(diameter=diameter),
                                     visualMaterial=vmat2,
                                     solidMaterial="BilliardBall",
                                     collision=true)),

    sphere3 = Object3D(parent=:(table.plate), fixedToParent=false,
                       translation=[-0.01, tableX/2-diameter/2-0.002, diameter/2+tableZ/2],
                       rotation=[pi/2, 0.0, 0.0],
                       feature=Solid(shape=Sphere(diameter=diameter),
                                     visualMaterial=vmat2,
                                     solidMaterial="BilliardBall",
                                     collision=true)),

    youbot1 = YouBot("world"),
    youbot2 = YouBot("world"),
    youbot3 = YouBot("world"),
)


modelParameters = Map(

    youbot1 = Map(
        servo1 = servoParameters1,
        servo2 = servoParameters2,
        servo3 = servoParameters3,
        servo4 = servoParameters4,
        servo5 = servoParameters5,
    ),

    youbot2 = Map(
        base = Map(
            rota = Par(value = :[0, 0, 180u"°"]),
            trans = [2*xPosTable,0.0,0.0],
        ),
        servo1 = servoParameters1,
        servo2 = servoParameters2,
        servo3 = servoParameters3,
        servo4 = servoParameters4,
        servo5 = servoParameters5,
    ),

    youbot3 = Map(
        base = Map(
            rota = Par(value = :[0, 0, 270u"°"]),
            trans = [xPosTable,xPosTable,0.0],
        ),
        servo1 = servoParameters1,
        servo2 = servoParameters2,
        servo3 = servoParameters3,
        servo4 = servoParameters4,
        servo5 = servoParameters5,
    ),

)

@time youbotModel = Setup | modelParameters

@time youbot = @instantiateModel(youbotModel, unitless=true, logCode=false, log=false, logTiming=false)

# logDetails=false, logStateSelection=false, logCode=true, logExecution=false, logTiming=false

stopTime = 5.0
testTime = 2.6
tolerance = 1e-6
requiredFinalStates = [-1.7446013429375096e-8, 1.651816677343481e-8, 0.00431720450734954, -0.05498074193509794, 0.018341528008953896, -0.2329217231959361, -2.5337916670764497e-5, 9.560550937643266e-6, 9.860473057871215e-8, -3.756045363017323e-8, 1.4450367055076657e-5, -0.16264410168532956, -0.19860989281569114, -0.08580109952299579, 0.0003301332361090111, -1.7328756504795486e-8, 1.6400895136727935e-8, 0.004317204558192761, -0.05498074198595921, 0.018341528035797503, -0.23292172322279361, -2.533792001918447e-5, 9.560554276577742e-6, 9.86055695046747e-8, -3.756129242358625e-8, 1.534667940338496e-5, -0.16264371307074668, -0.19860976131256988, -0.08580111118479883, 0.0003301361547878287, -1.726492051195686e-8, 1.6337066117997725e-8, 0.004317204675815967, -0.05498074210357591, 0.018341528043447162, -0.2329217232304452, -2.5337893689375363e-5, 9.560527946256025e-6, 9.861114278664596e-8, -3.756686574816713e-8, 1.5834644281321376e-5, -0.16264281402877054, -0.19860972388265669, -0.08580101960899753, 0.00033015554434589464, -0.03113425397916454, -0.11112132593533114, 0.029896003246475223, 0.0017939375234104287, -0.018298318501802153, 3.174006967011314e-6, 6.1194121263640335, -0.10326967252418215, -3.196394630247127, 0.7389184716377252, 0.07244237075015669, -1.4616472699948143e-15, -0.02118427218401483, -0.04298442706317084, 0.029894421659869684, -2.866553688899844e-10, -2.5955858582490327e-10, -2.0087317209515308e-6, 2.6909220669750424, 1.0504071869415947, 5.592638270787553, 7.519459074807808e-9, -8.305627052524455e-9, 3.5163250515460204e-15, 0.03267785643417389, 0.0861887351730743, 0.029894161337090563, 2.344729274767952e-15, 5.003377739303807e-16, -1.1378235558687383e-6, 2.942428579607958, -0.6653898194758066, 1.2901999776879125, -1.3910936201185747e-14, 6.551864900108236e-14, 1.0049236781320827e-26]
simulate!(youbot, stopTime=testTime, tolerance=tolerance, requiredFinalStates_atol=0.02, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

end
