module TwoStageRocket3DModule

using Modia3D

dt1 = 5.0
dt2 = 5.0
dt3 = 5.0
t1  = dt1         # stage1 and stage2 are seperating
t2  = t1 + dt2    # stage1 is deleted
t3  = t2 + dt3    # time when thrust of stage2 is zero.
f1max = 1e4       # maximum thrust of stage1
f2max = 0.2e4     # maximum thrust of stage2


function rocketProgram(actions)
    ActionAttach(actions, "stage1.top", "stage2.bottom")
    EventAfterPeriod(actions, dt1)
    ActionRelease(actions, "stage1.top")
    EventAfterPeriod(actions, dt2)
    ActionDelete(actions, "stage1.top")
    return nothing
end


linearThrust(time, t1, f1, t2, f2) =
    # it t1 <= time <= t2, return force at time linearly interpolated
    # through (t1,f1), (t2,f2) otherwise return 0.0
    if time >= t1 && time <= t2
        (f2-f1)/(t2-t1)*(time-t1) + f1
    else
        0.0
    end

thrustStage1(; time, objectApply, objectCoord) =
    Modia.SVector{3,Float64}(0.0, linearThrust(time, 0.0, f1max, t1, 0.0), 0.0)
thrustStage2(; time, objectApply, objectCoord) =
    Modia.SVector{3,Float64}(0.0, linearThrust(time, t1, f2max, t3, 0.0), 0.0)

# L: length of rocket stage
# d: diameter of rocket stage
# m: mass of rocket stage
# color: color of cylinder
RocketStage(; L=1.0, d=0.1, m=100.0, color="blue", thrustFunction,  translation) = Model(
    # Rocket stage is a cylinder
    body = Object3D(parent=:world, fixedToParent=false, assemblyRoot=true,
                    translation=translation,  feature=Solid(
                        massProperties=MassProperties(mass=m, Ixx=1.0, Iyy=1.0, Izz=1.0),
                        shape=Cylinder(axis=2, diameter=d, length=L),
                        visualMaterial=VisualMaterial(color=color, transparency=0.5))),
    bottom = Object3D(parent=:body, translation=:[0.0, -$L/2, 0.0], lockable=true),
	top    = Object3D(parent=:body, translation=:[0.0,  $L/2, 0.0], lockable=true),
    thrust = WorldForce(; objectApply=:bottom, forceFunction=thrustFunction)
)

TwoStageRocket = Model3D(
    world = Object3D(feature=Scene(enableContactDetection=false)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=1.0))),
    stage1 = RocketStage(L=2.0, d=0.2 , color="blue", thrustFunction=thrustStage1,
                translation=[0,1,0]),
    stage2 = RocketStage(L=1.0, d=0.15, color="red" , thrustFunction=thrustStage2,
                translation=[0,2.5,0]),

    modelActions = ModelActions(world=:world, actions=rocketProgram),
    currentAction = Var(hideResult=true),
    equations=:[
        currentAction = executeActions(modelActions)
    ],
)

rocket = @instantiateModel(TwoStageRocket, unitless=true)

simulate!(rocket, stopTime=15.0)

@usingModiaPlot
plot(rocket, [("stage2.body.translation[2]", "stage1.body.translation[2]" ),
              ("stage2.body.velocity[2]", "stage1.body.velocity[2]"),
              ("stage1.thrust.forceVector[2]", "stage2.thrust.forceVector[2]")])
end
