using OrderedCollections

appendSymbol(path::Nothing, name::Union{Symbol,Expr}) = name
appendSymbol(path         , name::Union{Symbol,Expr}) = :( $path.$name )
appendSymbol(path::String , name::Union{Symbol,Expr}) = path == "" ? string(name) : path*"."*string(name)

derSymbol(path::Nothing, name::Symbol) = :(der($name))
derSymbol(path         , name::Symbol) = :(der($path.$name))

nextMbsName(mbsi,i) = (Symbol("_mbs"*string(i+1)), mbsi, i+1)


"""
    buildCode = buildModia3D!(model, FloatType, TimeType, buildDict, modelPath; buildOption = "ComputeGeneralizedForces")

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
mbs1 = Modia3D.openModel3D!(instantiatedModel, modelPathAsString, ndofTotal, time)
mbs2 = Modia3D.setStatesRevolute!(mbs1, jointStatesRevolute...)
mbs3 = Modia3D.setStatesPrismatic!(mbs2, jointStatesPrismatic...)
mbs4 = Modia3D.setStatesFreeMotion!(mbs3, jointStatesFreeMotion...)
mbs5 = Modia3D.setStatesFreeMotion_isrot123!(mbs4, jointStatesFreeMotion_isrot123...)

if buildOption == "ComputeGeneralizedForces"
    # genForces := M(q)*qdd + h(q,qd)
    mbs6 = Modia3D.setAccelerationsRevolute!  (mbs5, jointAccelerationsRevolute...)
    mbs7 = Modia3D.setAccelerationsPrismatic!( mbs6, jointAccelerationsPrismatic...)
    mbs8 = Modia3D.setAccelerationsFreeMotion!(mbs7, jointAccelerationsFreeMotion2...) 
    mbs9 = Modia3D.computeGeneralizedForces!(  mbs8, _leq)
    (success1, jointForcesRevolute...)   = implicitDependency(Modia3D.getGenForcesRevolute(  mbs9, Val(NRevolute))   , jointAccelerationsRevolute...))
    (success2, jointForcesPrismatic...)  = implicitDependency(Modia3D.getGenForcesPrismatic( mbs9, Val(NPrismatic))  , jointAccelerationsPrismatic...)) 
    (success3, jointForcesFreeMotion...) = implicitDependency(Modia3D.getGenForcesFreeMotion(mbs9, Val(NFreeMotion2)), jointAccelerationsFreeMotion2...))
    success4 = Modia3D.closeModel3D!(mbs1, success1, success2, success3)
end
```
"""
function buildModia3D!(model::AbstractDict, FloatType::Type, TimeType::Type,
                       buildDict::OrderedCollections.OrderedDict{String,Any},
                       modelPath::Union{Expr,Symbol,Nothing}, 
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

    println("modelPath = $modelPath")
    
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

    i=1
    mbsi = :_mbs1
    mbs_equations = [ :($mbsi = Modia3D.openModel3D!(instantiatedModel, $modelPathAsString, $ndofTotal, time)) ]
    mbs_variables = Model()
    mbs_variables[mbsi] = Var(hideResult=true)
    if ndofTotal > 0
        if length(jointStatesRevolute) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
            push!(mbs_equations, :( $mbsi = Modia3D.setStatesRevolute!($mbsi_old, $(jointStatesRevolute...)) ))
            mbs_variables[mbsi] = Var(hideResult=true)
        end
        if length(jointStatesPrismatic) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
            push!(mbs_equations, :( $mbsi = Modia3D.setStatesPrismatic!($mbsi_old, $(jointStatesPrismatic...)) ))
            mbs_variables[mbsi] = Var(hideResult=true)            
        end    
        if length(jointStatesFreeMotion) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
            push!(mbs_equations, :( $mbsi = Modia3D.setStatesFreeMotion!($mbsi_old, $(jointStatesFreeMotion...)) ))
            mbs_variables[mbsi] = Var(hideResult=true)            
            (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
            push!(mbs_equations, :( $mbsi = Modia3D.setStatesFreeMotion_isrot123!($mbsi_old, $(jointStatesFreeMotion_isrot123...)) ))
            mbs_variables[mbsi] = Var(hideResult=true)
        end
    
        if buildOption == "ComputeGeneralizedForces"
            if length(jointAccelerationsRevolute) > 0
                (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
                push!(mbs_equations, :( $mbsi = Modia3D.setAccelerationsRevolute!($mbsi_old, $(jointAccelerationsRevolute...)) ))
                mbs_variables[mbsi] = Var(hideResult=true)                
            end
            if length(jointAccelerationsPrismatic) > 0
                (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
                push!(mbs_equations, :( $mbsi = Modia3D.setAccelerationsPrismatic!($mbsi_old, $(jointAccelerationsPrismatic...)) ))
                mbs_variables[mbsi] = Var(hideResult=true)                    
            end        
            if length(jointAccelerationsFreeMotion2) > 0
                (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
                push!(mbs_equations, :( $mbsi = Modia3D.setAccelerationsFreeMotion!($mbsi_old, $(jointAccelerationsFreeMotion2...)) ))
                mbs_variables[mbsi] = Var(hideResult=true)                    
            end
            (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
            push!(mbs_equations, :( $mbsi = Modia3D.computeGeneralizedForces!($mbsi_old, _leq_mode) )) 
            mbs_variables[mbsi] = Var(hideResult=true)                
        
            if length(jointAccelerationsRevolute) > 0
                push!(mbs_equations, :( ($(jointForcesRevolute...), ) = implicitDependency(Modia3D.getGenForcesRevolute($mbsi, Val($NRevolute)), $(jointAccelerationsRevolute...)) ))
                mbs_variables[mbsi] = Var(hideResult=true)                    
            end
            if length(jointAccelerationsPrismatic) > 0
                push!(mbs_equations, :( ($(jointForcesPrismatic...), ) = implicitDependency(Modia3D.getGenForcesPrismatic($mbsi, Val($NPrismatic)), $(jointAccelerationsPrismatic...)) ))
                mbs_variables[mbsi] = Var(hideResult=true)                    
            end
            if length(jointAccelerationsFreeMotion2) > 0
                NFreeMotion2 = 2*NFreeMotion
                push!(mbs_equations, :( ($(jointForcesFreeMotion2...), ) = implicitDependency(Modia3D.getGenForcesFreeMotion($mbsi), $(jointAccelerationsFreeMotion2...)) ))
                mbs_variables[mbsi] = Var(hideResult=true)                    
            end

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
    else
        # ndofTotal == 0
        mbsCode = mbs_variables | Model(equations = :[$(mbs_equations...)])
    end
    
    # Store info in buildDict
    buildDict[modelPathAsString] = Modia3D.Composition.MultibodyBuild{FloatType,TimeType}(modelPathAsString, Modia.splitPath(modelPath), revoluteIndices, prismaticIndices, freeMotionIndices)
    return mbsCode
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
