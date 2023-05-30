using OrderedCollections

appendSymbol(path::Nothing, name::Union{Symbol,Expr}) = name
appendSymbol(path         , name::Union{Symbol,Expr}) = :( $path.$name )
appendSymbol(path::String , name::Union{Symbol,Expr}) = path == "" ? string(name) : path*"."*string(name)

derSymbol(path::Nothing, name::Symbol) = :(der($name))
derSymbol(path         , name::Symbol) = :(der($path.$name))

nextMbsName(mbs,i,mbsi) = (Symbol(mbs*string(i+1)), mbsi, i+1)


"""
    buildCode = build_Model3D!(model, FloatType, TimeType, unitless, ID, modelPath; buildOption = "ComputeGeneralizedForces")

Generate and return the buildCode for a Modia3D model.

# Arguments

- `model`: Modia Model that contains Modia3D elements.
- `FloatType`, `TimeType`: Types used when instantiating `SimulationModel{FloatType,TimeType}
- `modelPath`: Path upto `model`. Path is a Symbol or Expr (such as :( a.b.c )) or nothing, if at the root.
- `buildDict`: Dictionary, that will be stored in SimulationModel. An initial instance of Modia3D.Composition.MultibodyBuild{FloatType,TimeType}
               is stored in `buildDict` with key `string(modelPath)`, containing info about the generated code, in particular the joint type, path and
               order of the joint variables in the function calls present in the returned `buildCode`.
- `buildOption`: Code generation method. Possible values: `"ComputeGeneralizedForces"` (and in the future: `"ComputeGeneralizedAccelerations"`).


# Code Generation

This function returns the following type of code:

```
qdd_hidden = Var(hideResult=true, start=[]),
success = Var(hideResult=true),
mbs1 = Var(hideResult=true),
mbs2 = Var(hideResult=true),
mbs3 = Var(hideResult=true),
mbs4 = Var(hideResult=true),
mbs5 = Var(hideResult=true),
mbs6 = Var(hideResult=true),
mbs7 = Var(hideResult=true),
mbs8 = Var(hideResult=true),
mbs9 = Var(hideResult=true),
equations = :[
    mbs1 = Modia3D.openModel3D!(instantiatedModel, modelPathAsString, _x, time)
    mbs2 = Modia3D.setStatesRevolute!(mbs1, jointStatesRevolute...)
    mbs3 = Modia3D.setStatesPrismatic!(mbs2, jointStatesPrismatic...)
    mbs4 = Modia3D.setStatesFreeMotion!(mbs3, jointStatesFreeMotion...)
    mbs5 = Modia3D.setStatesFreeMotion_isrot123!(mbs4, jointStatesFreeMotion_isrot123...)

    if buildOption == "ComputeGeneralizedForces"
        # genForces := M(q)*qdd + h(q,qd)
        mbs6 = Modia3D.setAccelerationsRevolute! (   mbs5, jointAccelerationsRevolute...)
        mbs7 = Modia3D.setAccelerationsPrismatic!(   mbs6, jointAccelerationsPrismatic...)
        mbs8 = Modia3D.setAccelerationsFreeMotion!(  mbs7, jointAccelerationsFreeMotion2...)
        mbs9 = Modia3D.setAccelerationsHiddenJoints!(mbs8, qdd_hidden)
        mbs10 = Modia3D.computeGeneralizedForces!(mbs9, _leq)
        (jointForcesRevolute...)  = implicitDependency(Modia3D.getGenForcesRevolute(    mbs10, Val(NRevolute))   , jointAccelerationsRevolute...))
        (jointForcesPrismatic...) = implicitDependency(Modia3D.getGenForcesPrismatic(   mbs10, Val(NPrismatic))  , jointAccelerationsPrismatic...))
        (0.0, 0.0, ...)           = implicitDependency(Modia3D.getGenForcesFreeMotion(  mbs10, Val(NFreeMotion2)), jointAccelerationsFreeMotion2...))
        0.0                       =                    Modia3D.getGenForcesHiddenJoints(mbs10, qdd_hidden)
        success = Modia3D.setHiddenStatesDerivatives!(instantiatedModel, mbs10)
    end
]
```
"""
function build_Model3D!(model::AbstractDict, FloatType::Type, TimeType::Type, unitless::Bool,
                       ID, modelPath::Union{Expr,Symbol,Nothing},
                       buildOption::String = "ComputeGeneralizedForces")   # ComputeJointAccelerations, ComputeJointAccelerationsOn
    @assert(buildOption == "ComputeGeneralizedForces")
    jointInfo = []
    getJointInfo!(model, jointInfo)

    #=
       # not yet implemented:
       buildOption == "ComputeGeneralizedAccelerations": # qdd := M(q)\(h(q,qd) - u)
                  _qdd  = Modia3D.getJointResiduals_method3!(_mbs2, $(jointForces1...))
                  _qdd[1] = ...
                  _qdd[2] = ...


                         equations = :[_mbs1 = Modia3D.initJoints!(_id, instantiatedModel, $ndofTotal, time)
                                      _mbs2 = Modia3D.setJointStates1!(_mbs1, $(jointStates1...))
                                      _mbs3 = Modia3D.setJointAccelerations1!(_mbs2, $(jointAcc1...))
                                      (dummy,) = implicitDependency(Modia3D.getJointResiduals_method2!(_mbs3, _leq_mode), $(jointAcc1...))
                                     ]
                        )


   =#

    ndofTotal = 0

    NRevolute = 0
    jointStatesRevolute        = Expr[]
    jointForcesRevolute        = []
    jointAccelerationsRevolute = []
    revoluteIndices            = OrderedCollections.OrderedDict{String,Int}()

    NPrismatic = 0
    jointStatesPrismatic        = Expr[]
    jointForcesPrismatic        = []
    jointAccelerationsPrismatic = []
    prismaticIndices            = OrderedCollections.OrderedDict{String,Int}()

    NFreeMotion = 0
    jointStatesFreeMotion         = Expr[]
    jointForcesFreeMotion2         = []
    jointAccelerationsFreeMotion2  = []
    jointStatesFreeMotion_isrot123 = Expr[]
    freeMotionIndices              = OrderedCollections.OrderedDict{String,Int}()

    #println("modelPath = $modelPath")
    modelPathAsString = isnothing(modelPath) ? "" : string(modelPath)

    i=1
    for joint in jointInfo
        path      = joint.path
        jointType = joint.jointType

        if jointType == :Revolute || jointType == :RevoluteWithFlange
            ndofTotal += 1
            NRevolute += 1
            revoluteIndices[appendSymbol(modelPathAsString,path)] = NRevolute
            push!(jointStatesRevolute, appendSymbol(path, :phi))
            push!(jointStatesRevolute, appendSymbol(path, :w))
            if jointType == :RevoluteWithFlange
                push!(jointForcesRevolute, appendSymbol(appendSymbol(path, :flange), :tau))
            else
                push!(jointForcesRevolute, 0.0 )
            end
            if buildOption == "ComputeGeneralizedForces"
                push!(jointAccelerationsRevolute, derSymbol(path, :w))
            elseif buildOption == "ComputeGeneralizedAccelerations"
                der_w = derSymbol(path, :w)
                push!(jointAccelerationsRevolute, :( $der_w = _qdd[$i]) )
                i += 1
            end

        elseif jointType == :Prismatic || jointType == :PrismaticWithFlange
            ndofTotal  += 1
            NPrismatic += 1
            prismaticIndices[appendSymbol(modelPathAsString,path)] = NPrismatic
            push!(jointStatesPrismatic, appendSymbol(path, :s))
            push!(jointStatesPrismatic, appendSymbol(path, :v))
            if jointType == :PrismaticWithFlange
                push!(jointForcesPrismatic, appendSymbol(appendSymbol(path, :flange), :f))
            else
                push!(jointForcesPrismatic, :(0.0))
            end
            if buildOption == "ComputeGeneralizedForces"
                push!(jointAccelerationsPrismatic, derSymbol(path, :v))
            elseif buildOption == "ComputeGeneralizedAccelerations"
                der_v = derSymbol(path, :v)
                push!(jointAccelerationsPrismatic, :( $der_v = _qdd[$i]) )
                i += 1
            end

        elseif jointType == :FreeMotion
            ndofTotal += 6
            NFreeMotion += 1
            freeMotionIndices[appendSymbol(modelPathAsString,path)] = NFreeMotion
            push!(jointStatesFreeMotion, appendSymbol(path, :r))
            push!(jointStatesFreeMotion, appendSymbol(path, :v))
            push!(jointStatesFreeMotion, appendSymbol(path, :rot))
            push!(jointStatesFreeMotion, appendSymbol(path, :w))
            push!(jointStatesFreeMotion_isrot123, appendSymbol(path, :isrot123))
            push!(jointForcesFreeMotion2, :(0.0))
            push!(jointForcesFreeMotion2, :(0.0))

            if buildOption == "ComputeGeneralizedForces"
                push!(jointAccelerationsFreeMotion2, derSymbol(path, :v))
                push!(jointAccelerationsFreeMotion2, derSymbol(path, :w))
            elseif buildOption == "ComputeGeneralizedAccelerations"
                der_v = derSymbol(path, :v)
                der_w = derSymbol(path, :w)
                push!(jointAccelerationsFreeMotion2, :( $der_v = _qdd[$i]) )
                push!(jointAccelerationsFreeMotion2, :( $der_v = _qdd[$i+1]) )
                i += 2
            end

        else
            error("\nJoint type $jointType in submodel with path $path is not known.")
        end
    end

    if modelPathAsString == ""
        mbs       = "_mbs"
        genForces = :_genForces
    else
        mbs       = modelPathAsString * "._mbs"
        genForces = Symbol(modelPathAsString * "._genForces")
    end
    i=0
    (mbsi, mbsi_old, i) = nextMbsName(mbs, i, :_dummy)
    mbs_equations = [ :($mbsi = Modia3D.openModel3D!(instantiatedModel, $ID, _x, time)) ]
    mbs_variables = Model(qdd_hidden = Var(hideResult=true, start=[]),
                          success = Var(hideResult=true))
    mbs_variables[mbsi] = Var(hideResult=true)

    if length(jointStatesRevolute) > 0
        (mbsi, mbsi_old, i) = nextMbsName(mbs, i, mbsi)
        push!(mbs_equations, :( $mbsi = Modia3D.setStatesRevolute!($mbsi_old, $(jointStatesRevolute...)) ))
        mbs_variables[mbsi] = Var(hideResult=true)
    end
    if length(jointStatesPrismatic) > 0
        (mbsi, mbsi_old, i) = nextMbsName(mbs, i, mbsi)
        push!(mbs_equations, :( $mbsi = Modia3D.setStatesPrismatic!($mbsi_old, $(jointStatesPrismatic...)) ))
        mbs_variables[mbsi] = Var(hideResult=true)
    end
    if length(jointStatesFreeMotion) > 0
        (mbsi, mbsi_old, i) = nextMbsName(mbs, i, mbsi)
        push!(mbs_equations, :( $mbsi = Modia3D.setStatesFreeMotion!($mbsi_old, $(jointStatesFreeMotion...)) ))
        mbs_variables[mbsi] = Var(hideResult=true)
        (mbsi, mbsi_old, i) = nextMbsName(mbs, i, mbsi)
        push!(mbs_equations, :( $mbsi = Modia3D.setStatesFreeMotion_isrot123!($mbsi_old, $(jointStatesFreeMotion_isrot123...)) ))
        mbs_variables[mbsi] = Var(hideResult=true)
    end

    if buildOption == "ComputeGeneralizedForces"
        if length(jointAccelerationsRevolute) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbs, i, mbsi)
            push!(mbs_equations, :( $mbsi = Modia3D.setAccelerationsRevolute!($mbsi_old, $(jointAccelerationsRevolute...)) ))
            mbs_variables[mbsi] = Var(hideResult=true)
        end
        if length(jointAccelerationsPrismatic) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbs, i, mbsi)
            push!(mbs_equations, :( $mbsi = Modia3D.setAccelerationsPrismatic!($mbsi_old, $(jointAccelerationsPrismatic...)) ))
            mbs_variables[mbsi] = Var(hideResult=true)
        end
        if length(jointAccelerationsFreeMotion2) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbs, i, mbsi)
            push!(mbs_equations, :( $mbsi = Modia3D.setAccelerationsFreeMotion!($mbsi_old, $(jointAccelerationsFreeMotion2...)) ))
            mbs_variables[mbsi] = Var(hideResult=true)
        end

        unknowns = []
        append!(unknowns, jointAccelerationsRevolute, jointForcesRevolute, jointAccelerationsPrismatic, jointForcesPrismatic, jointAccelerationsFreeMotion2)
        push!(mbs_equations, :( $genForces = implicitDependency(Modia3D.computeGeneralizedForces!($mbsi, qdd_hidden, _leq_mode), $unknowns) ))
        mbs_variables[mbsi] = Var(hideResult=true)
        mbs_variables[genForces] = Var(hideResult=true)

        i = 0
        for j=1:length(jointAccelerationsRevolute)
            i += 1
            push!(mbs_equations, :( $(jointForcesRevolute[j]) = implicitDependency($genForces[$i], $mbsi, $unknowns)) )
        end
        for j=1:length(jointAccelerationsPrismatic)
            i += 1
            push!(mbs_equations, :( $(jointForcesPrismatic[j]) = implicitDependency($genForces[$i], $mbsi, $unknowns)) )
        end
        if length(jointAccelerationsFreeMotion2) > 0
            NFreeMotion2 = 2*NFreeMotion
            push!(mbs_equations, :( ($(jointForcesFreeMotion2...), ) = implicitDependency(Modia3D.getGenForcesFreeMotion($mbsi, $genForces), $(jointAccelerationsFreeMotion2...)) ))
        end
        push!(mbs_equations, :( 0.0 = Modia3D.getGenForcesHiddenJoints($mbsi, $genForces, qdd_hidden) ) )
        push!(mbs_equations, :( success = Modia3D.setHiddenStatesDerivatives!(instantiatedModel, $mbsi, $genForces) ) )

        mbsCode = mbs_variables | Model(equations = :[$(mbs_equations...)])

    elseif method == "ComputeJointAccelerations"
        mbsCode = Model(_id = rand(Int),
                        _qdd = Var(start = zeros(ndofTotal)),
                        equations = :[_mbs1 = Modia3D.initJoints!(_id, instantiatedModel, $ndofTotal, time)
                                    _mbs2 = Modia3D.setJointStates1!(_mbs1, $(jointStates1...))
                                    _qdd = Modia3D.getJointResiduals_method3!(_mbs2, $(jointForces1...))
                                    $(jointAcc1...)
                                    ]
                        )
    else
        @error "Error should not occur (buildOption = $buildOption)"
    end

    return ( model|mbsCode, Modia3D.Composition.MultibodyBuild{FloatType,TimeType}(modelPathAsString, revoluteIndices, prismaticIndices, freeMotionIndices) )
end


"""
    getJointInfo!(model, jointInfo; path=nothing)

Recursively traverse the OrderedDict `model` and search for key :_jointType in a dictionary
that has key :_constructor and return a vector of tuples [(path_i, _jointType_i)]
A tuple contains the path to a joint (where the path is defined as
quoted expression, for example :(a.b.c)), and a tuple _jointInfo, for example :Revolute.
"""
function getJointInfo!(model, jointInfo; path=nothing)::Nothing
    if haskey(model, :_constructor)
        constructor = model[:_constructor]
        if typeof(constructor) <: OrderedDict && haskey(constructor, :_jointType)
            if haskey(model, :_rotName)
                # Hack to provide the full path name of FreeMotion.rot to the FreeMotion object
                model[:_rotName] = string(path)*".rot"
            end
            push!(jointInfo, (path=path, jointType=constructor[:_jointType]))
            return
        end
    end

    for (key,value) in model
        if typeof(value) <: OrderedDict
            getJointInfo!(value, jointInfo; path=appendSymbol(path, key))
        end
    end
    return nothing
end


struct AnimationHistoryElement
    positions::Vector{  SVector{3,Float64}}    # abs. Object3D position
    quaternions::Vector{SVector{4,Float64}}    # abs. Object3D quaternion
end


"""
    animationHistory = get_animationHistory(instantiatedModel, modelPathAsString)

After a simulation, return an ordered dictionary of the positions and orientations of all visual Object3D
present in `instantiatedModel` with respect to the `modelPathAsString` of a `Model3D(..)` model.
The time vector is provided as `animationHistory["time"]`.
The animation history of an Object3D with name `"a.b.c"` is provided in the form:

```julia
animationHistory["a.b.c"] = OrdereDict{String,Any}("position"   => pos,
                                                   "quaternion" => quat)

#= Example
pos = [[1.0, 2.0, 3.0],
       [1.1, 2.1, 3.1],  # Absolute position [1.1, 2.2, 3.1] at time instant 2.
       [1.2, 2.2, 3.2],
       [1.3, 2.3, 3.3] ...]

quat = [[0.10, 0.20, 0.30, 0.86],
        [0.11, 0.21, 0.31, 0.8477], ...]   # Quaternions [0.11, 0.21, 0.31, 0.8477]
                                           # to rotate from world to Object3D at time instant 2.
=#
```
"""
function get_animationHistory(instantiatedModel::Modia.SimulationModel{FloatType,TimeType},
                              modelPathAsString::String; log::Bool = true)::Union{OrderedDict{String,Any},Nothing} where {FloatType,TimeType}

    mbs::Modia3D.Composition.MultibodyData{FloatType,TimeType} = instantiatedModel.buildDict[modelPathAsString].mbs
    scene = mbs.scene
    allVisuElements = scene.allVisuElements
    animation = scene.animation
    animationHistory = OrderedDict{String,Any}()

    if length(allVisuElements) > 0 && scene.provideAnimationData
        if log
            println("get_animationHistory(..): Return animation history of ", length(allVisuElements), " Object3Ds at ",
                    length(animation), " time instants")
        end

        timeVector = Float64[]
        for obj in animation
            push!(timeVector, obj.time)
        end
        animationHistory["time"] = timeVector

        for (iobj,obj) in enumerate(allVisuElements)
            positions   = SVector{3,Float64}[]
            quaternions = SVector{4,Float64}[]
            for animationStep in animation
                push!(positions  , animationStep.objectData[iobj].position)
                push!(quaternions, animationStep.objectData[iobj].quaternion)
            end
            animationHistory[obj.path] = AnimationHistoryElement(positions, quaternions)
        end
        return animationHistory
    else
        println("\nget_animationHistory(..):\n  No animation history stored during simulation.",
                "\n  Use Object3D(feature=Scene(provideAnimationHistory=true))!!!")
        return nothing
    end
end
