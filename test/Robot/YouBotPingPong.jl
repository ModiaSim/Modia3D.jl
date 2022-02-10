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
nullRot = nothing

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

#### ----------- Path Planning ------------------
referencePath1 = Modia3D.PathPlanning.ReferencePath(
    names = ["angle1", "angle2", "angle3", "angle4", "angle5", "gripper"],
    position = [0.0,     0.0,     pi/2,   0.0,    0.0,   0.0],
    v_max =    [2.68512, 2.68512, 4.8879, 5.8997, 5.8997, 1.0],
    a_max =    [1.5, 1.5, 1.5, 1.5, 1.5, 0.5])
#
Modia3D.PathPlanning.ptpJointSpace(referencePath = referencePath1, positions =
    [0.0 0.0 pi/2     0.0 0.0 0.0;
    0.0  0.3 pi/2-0.3 0.0 0.0 0.0;
    0.0  0.0 0.0      0.0 0.0 0.0])

getReferencePath() = referencePath1

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
                    initRefPos = referencePath1.position[1],
                    trans = translation1,
                    rota = Par(value = :(rotation1))
)

linkParameters2 = Map(
                    parent1 = Par(value = :(link1.obj2)),
                    featureBody = featureBody2, #Par(value = :featureBody2),
                    m = m2,
                    initRefPos = referencePath1.position[2],
                    trans = translation2,
                    rota = Par(value = :(rotation2))
)

linkParameters3 = Map(
                    parent1 = Par(value = :(link2.obj2)),
                    featureBody = featureBody3, # Par(value = :featureBody3),
                    m = m3,
                    initRefPos = referencePath1.position[3],
                    trans = translation3,
                    rota = Par(value = :(rotation3))
)

linkParameters4 = Map(
                    parent1 = Par(value = :(link3.obj2)),
                    featureBody = featureBody4, # Par(value = :featureBody4),
                    m = m4,
                    initRefPos = referencePath1.position[4],
                    trans = translation4,
                    rota = Par(value = :(nullRot))
)

linkParameters5 = Map(
                    parent1 = Par(value = :(link4.obj2)),
                    featureBody = featureBody5, # Par(value = :featureBody5),
                    m = m5,
                    initRefPos = referencePath1.position[5],
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

YouBot = Model(
    base = Base,

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
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 1)), w=Var(init=0.0)),
    rev2 = RevoluteWithFlange(obj1 = :(link2.obj1), obj2 = :(link2.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 1)), w=Var(init=0.0)),
    rev3 = RevoluteWithFlange(obj1 = :(link3.obj1), obj2 = :(link3.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 1)), w=Var(init=0.0)),
    rev4 = RevoluteWithFlange(obj1 = :(link4.obj1), obj2 = :(link4.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 1)), w=Var(init=0.0)),
    rev5 = RevoluteWithFlange(obj1 = :(link5.obj1), obj2 = :(link5.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 1)), w=Var(init=0.0)),

    servo1 = Servo,
    servo2 = Servo,
    servo3 = Servo,
    servo4 = Servo,
    servo5 = Servo,

    equations=:[
        refPath = calculateRobotMovement(getReferencePath(), instantiatedModel),
        servo1.refLoadAngle = getRefPathPosition(refPath, 1),
        servo2.refLoadAngle = getRefPathPosition(refPath, 2),
        servo3.refLoadAngle = getRefPathPosition(refPath, 3),
        servo4.refLoadAngle = getRefPathPosition(refPath, 4),
        servo5.refLoadAngle = getRefPathPosition(refPath, 5)
    ],

    connect = :[
        (servo1.flange, rev1.flange)
        (servo2.flange, rev2.flange)
        (servo3.flange, rev3.flange)
        (servo4.flange, rev4.flange)
        (servo5.flange, rev5.flange)
        ]
)

Setup = Model(
    gravField = UniformGravityField(g=9.81, n=[0,0,-1]),
    world = Object3D(feature=Scene(gravityField = :gravField, visualizeFrames=false, defaultFrameLength=0.1, enableContactDetection=true, elasticContactReductionFactor=1e-4, animationFile="YouBotPingPong.json")),

    # worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length = 0.5))),

    table = Table,
    ground = Ground,

    sphere1 = Object3D(
        feature=Solid(shape=Sphere(diameter=diameter), visualMaterial=vmat2, solidMaterial="BilliardBall", collision=true)),

    free1 = FreeMotion(obj1=:(table.plate), obj2=:sphere1, r=Var(init=[-tableX/2+diameter/2+0.001, 0.0002, diameter/2+tableZ/2]), rot=Var(init=[pi/2, 0.0, 0.0])),

    sphere2 = Object3D(
        feature=Solid(shape=Sphere(diameter=diameter), visualMaterial=vmat2, solidMaterial="BilliardBall", collision=true)),

    free2 = FreeMotion(obj1=:(table.plate), obj2=:sphere2, r=Var(init=[tableX/2-diameter/2-0.0001, -0.003, diameter/2+tableZ/2]), rot=Var(init=[pi/2, 0.0, 0.0])),

    sphere3 = Object3D(
        feature=Solid(shape=Sphere(diameter=diameter), visualMaterial=vmat2, solidMaterial="BilliardBall", collision=true)),

    free3 = FreeMotion(obj1=:(table.plate), obj2=:sphere3, r=Var(init=[-0.01, tableX/2-diameter/2-0.002, diameter/2+tableZ/2]), rot=Var(init=[pi/2, 0.0, 0.0])),

    youbot1 = YouBot,
    youbot2 = YouBot,
    youbot3 = YouBot,
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

@time youbotModel = buildModia3D(Setup) | modelParameters

@time youbot = @instantiateModel(youbotModel, unitless=true, logCode=false, log=false, logTiming=false)

# logDetails=false, logStateSelection=false, logCode=true, logExecution=false, logTiming=false

stopTime = 5.0
testTime = 2.6
tolerance = 1e-6
requiredFinalStates = [-0.03158747043096268, -0.10649074184772618, 0.029895517033172142, 0.0015801764831357115, -0.016103870180510103, 3.0647332775854547e-6, 5.931626393118782, -0.12111882537425132, -3.1998834006615127, -0.6434579376126508, -0.09754820912048147, -0.056751608359316544, -0.018177570782297342, -0.04024642751149416, 0.029895831658455524, 7.725182548591907e-14, 7.009575071513706e-14, -1.5832876599015842e-6, 2.660614917159462, 0.9432946129122108, 5.493556660598091, 1.059276838819331e-12, -1.5024971829819885e-12, -2.0515616279450872e-12, 0.030984374457220996, 0.08325328402030487, 0.029894964905552535, -1.0155184391451235e-15, 3.0723209984482187e-16, -9.459296674070217e-7, 3.069567335180294, -0.5911836947757584, 1.3138620051588528, 2.3812055360652834e-14, 1.1960425669561842e-14, 5.9686650805787834e-15, -1.1074562488447844e-6, -1.570828718718346e-6, 0.004318767690961195, -0.054981259608387505, 0.01834152242400252, -0.23292171802704026, -2.5341492401393595e-5, 9.563867635374948e-6, 9.861985469938779e-8, -3.7574791174841696e-8, -0.008203276554541334, 0.08427818457130207, -0.19863801640195203, -0.0858138644731111, 0.0003301870168371117, -1.1073726615546118e-6, -1.5709123053952306e-6, 0.004318767670170559, -0.054981259587595546, 0.01834152240597686, -0.23292171800901154, -2.534150403358706e-5, 9.563879270118567e-6, 9.862060174961774e-8, -3.757553823626217e-8, -0.00820263760817991, 0.08427802565969766, -0.1986381047108251, -0.08581390493294977, 0.0003301896157919179, -1.1076292439524772e-6, -1.5706557245991997e-6, 0.004318767741888795, -0.054981259659317175, 0.018341522413747244, -0.23292171801678854, -2.5341477333742316e-5, 9.563852565159495e-6, 9.862602231487768e-8, -3.7580958770311354e-8, -0.008204598936583091, 0.0842785738482444, -0.19863806667104147, -0.08581381206211312, 0.00033020847402692944]
simulate!(youbot, stopTime=testTime, tolerance=tolerance, requiredFinalStates_atol=0.001, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

end
