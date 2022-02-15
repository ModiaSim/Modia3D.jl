using OrderedCollections

appendSymbol(path::Nothing, name::Symbol) = name
appendSymbol(path         , name::Symbol) = :( $path.$name )

derSymbol(path::Nothing, name::Symbol) = :(der($name))
derSymbol(path         , name::Symbol) = :(der($path.$name))

nextMbsName(mbsi,i) = (Symbol("_mbs"*string(i+1)), mbsi, i+1)


"""
    buildModia3D(model; method = :ComputeGeneralizedForces)

Generate and return the following type of code:

```
model | Model(_id = rand(Int),
              equations = :[...])
```

# Arguments

- `model`: Modia model
- `method`: Code generation method. Possible values: `:ComputeGeneralizedForces` (and in the future: `:ComputeGeneralizedAccelerations`).
"""
function buildModia3D(model; method = :ComputeGeneralizedForces)   # :ComputeJointAccelerations, :ComputeJointAccelerationsOn
    @assert(method == :ComputeGeneralizedForces)
    jointInfo = []
    getJointInfo!(model, jointInfo, nothing)

    if length(jointInfo) == 0
        ndofTotal = 0    # what to do???
        args = nothing
        error("No joints present ->  so no states")
    end

    @show method

    #=
     
    method == :ComputeGeneralizedForces   # u := M(q)*qdd + h(q,qd)
    
                # Called outside of the linear equation system
                mbs1 = Modia3D.initJoints!(_id, instantiatedModel, $ndofTotal, time)
                mbs2 = Modia3D.setStatesRevolute!(mbs1, $(jointStatesRevolute...))
                mbs3 = Modia3D.setStatesPrismatic!(mbs2, $(jointStatesPrismatic...))
                mbs4 = Modia3D.setStatesFreeMotion!(mbs3, $(jointStatesFreeMotion2...))
                mbs5 = Modia3D.setStatesFreeMotion_isrot123!(mbs4, $(jointStatesFreeMotion_isrot123...))
                
                tau1 = <...>
                tau2 = <...>
                f1   = <...>
                f2   = <...>
                <...>
                
                # Called inside the linear equation system, after transforming getGenForcesXXX into residual form
                mbs6 = Modia3D.setAccelerationsRevolute!  (mbs5, $(jointAccelerationsRevolute...))
                mbs7 = Modia3D.setAccelerationsPrismatic!( mbs6, $(jointAccelerationsPrismatic...))
                mbs8 = Modia3D.setAccelerationsFreeMotion!(mbs7, $(jointAccelerationsFreeMotion2...)) 
                mbs9 = Modia3D.computeGeneralizedForces!(  mbs8, _leq)
                
                $(jointForcesRevolute...)   = implicitDependency(Modia3D.getGenForcesRevolute(  mbs9, Val(NRevolute))   , $(jointAccelerationsRevolute...))
                $(jointForcesPrismatic...)  = implicitDependency(Modia3D.getGenForcesPrismatic( mbs9, Val(NPrismatic))  , $(jointAccelerationsPrismatic...)) 
                $(jointForcesFreeMotion...) = implicitDependency(Modia3D.getGenForcesFreeMotion(mbs9, Val(NFreeMotion2)), $(jointAccelerationsFreeMotion2...))   

       # not yet implemented:
       method == :ComputeGeneralizedAccelerations: # qdd := M(q)\(h(q,qd) - u)
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

    NPrismatic = 0
    jointStatesPrismatic        = Expr[]
    jointForcesPrismatic        = []
    jointAccelerationsPrismatic = []
 
    NFreeMotion2 = 0 
    jointStatesFreeMotion2         = Expr[]
    jointForcesFreeMotion2         = []
    jointAccelerationsFreeMotion2  = []
    jointStatesFreeMotion_isrot123 = Expr[]
    
    i=1
    for joint in jointInfo
        path      = joint.path
        jointType = joint.jointType

        if jointType == :Revolute || jointType == :RevoluteWithFlange
            ndofTotal += 1
            NRevolute += 1
            push!(jointStatesRevolute, appendSymbol(path, :phi))
            push!(jointStatesRevolute, appendSymbol(path, :w))
            if jointType == :RevoluteWithFlange
                push!(jointForcesRevolute, appendSymbol(appendSymbol(path, :flange), :tau))
            else
                push!(jointForcesRevolute, 0.0 )
            end
            if method == :ComputeGeneralizedForces
                push!(jointAccelerationsRevolute, derSymbol(path, :w))
            elseif method == :ComputeGeneralizedAccelerations
                der_w = derSymbol(path, :w)
                push!(jointAccelerationsRevolute, :( $der_w = _qdd[$i]) )
                i += 1
            end

        elseif jointType == :Prismatic || jointType == :PrismaticWithFlange
            ndofTotal  += 1
            NPrismatic += 1
            push!(jointStatesPrismatic, appendSymbol(path, :s))
            push!(jointStatesPrismatic, appendSymbol(path, :v))
            if jointType == :PrismaticWithFlange
                push!(jointForcesPrismatic, appendSymbol(appendSymbol(path, :flange), :f))
            else
                push!(jointForcesPrismatic, :(0.0))
            end
            if method == :ComputeGeneralizedForces
                push!(jointAccelerationsPrismatic, derSymbol(path, :v))
            elseif method == :ComputeGeneralizedAccelerations
                der_v = derSymbol(path, :v)
                push!(jointAccelerationsPrismatic, :( $der_v = _qdd[$i]) )
                i += 1
            end

        elseif jointType == :FreeMotion
            ndofTotal += 6
            NFreeMotion2 += 2  # Number of objects in the returned tuple of getGenForcesFreeMotion
            push!(jointStatesFreeMotion2, appendSymbol(path, :r))
            push!(jointStatesFreeMotion2, appendSymbol(path, :rot))
            push!(jointStatesFreeMotion2, appendSymbol(path, :v))
            push!(jointStatesFreeMotion2, appendSymbol(path, :w))
            push!(jointStatesFreeMotion_isrot123, appendSymbol(path, :isrot123))
            push!(jointForcesFreeMotion2, :(0.0))
            push!(jointForcesFreeMotion2, :(0.0))

            if method == :ComputeGeneralizedForces
                push!(jointAccelerationsFreeMotion2, derSymbol(path, :v))
                push!(jointAccelerationsFreeMotion2, derSymbol(path, :w))
            elseif method == :ComputeGeneralizedAccelerations
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
    mbs_equations = [ :($mbsi = Modia3D.initJoints!(_id, instantiatedModel, $ndofTotal, time)) ]
    
    if length(jointStatesRevolute) > 0
        (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
        push!(mbs_equations, :( $mbsi = Modia3D.setStatesRevolute!($mbsi_old, $(jointStatesRevolute...)) ))
    end
    if length(jointStatesPrismatic) > 0
        (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
        push!(mbs_equations, :( $mbsi = Modia3D.setStatesPrismatic!($mbsi_old, $(jointStatesPrismatic...)) ))
    end    
    if length(jointStatesFreeMotion2) > 0
        (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
        push!(mbs_equations, :( $mbsi = Modia3D.setStatesFreeMotion!($mbsi_old, $(jointStatesFreeMotion2...)) ))
        (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
        push!(mbs_equations, :( $mbsi = Modia3D.setStatesFreeMotion_isrot123!($mbsi_old, $(jointStatesFreeMotion_isrot123...)) ))
    end

    if method == :ComputeGeneralizedForces
        if length(jointAccelerationsRevolute) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
            push!(mbs_equations, :( $mbsi = Modia3D.setAccelerationsRevolute!($mbsi_old, $(jointAccelerationsRevolute...)) ))
        end
        if length(jointAccelerationsPrismatic) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
            push!(mbs_equations, :( $mbsi = Modia3D.setAccelerationsPrismatic!($mbsi_old, $(jointAccelerationsPrismatic...)) ))
        end        
        if length(jointAccelerationsFreeMotion2) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
            push!(mbs_equations, :( $mbsi = Modia3D.setAccelerationsFreeMotion!($mbsi_old, $(jointAccelerationsFreeMotion2...)) ))
        end
        (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
        push!(mbs_equations, :( $mbsi = Modia3D.computeGeneralizedForces!($mbsi_old, _leq_mode) ))  
       
        if length(jointAccelerationsRevolute) > 0
            push!(mbs_equations, :( ($(jointForcesRevolute...), ) = implicitDependency(Modia3D.getGenForcesRevolute($mbsi, Val($NRevolute)), $(jointAccelerationsRevolute...)) ))
        end
        if length(jointAccelerationsPrismatic) > 0
            push!(mbs_equations, :( ($(jointForcesPrismatic...), ) = implicitDependency(Modia3D.getGenForcesPrismatic($mbsi, Val($NPrismatic)), $(jointAccelerationsPrismatic...)) ))
        end
        if length(jointAccelerationsFreeMotion2) > 0
            push!(mbs_equations, :( ($(jointForcesFreeMotion2...), ) = implicitDependency(Modia3D.getGenForcesFreeMotion($mbsi, Val($NFreeMotion2)), $(jointAccelerationsFreeMotion2...)) ))
        end
      
        #push!(mbs_equations, :( (dummy,) = implicitDependency(Modia3D.getJointResiduals_method2!($mbsi, _leq_mode), $(jointAcc1...), $(jointAcc6...)) ))
        #push!(mbs_equations, :( ($(jointForces1...), $(jointForces6...)) = implicitDependency(Modia3D.getJointResiduals_method2!($mbsi, _leq_mode), $(jointAcc1...), $(jointAcc6...)) ))

        mbsCode = Model(_id = rand(Int),
                        equations = :[$(mbs_equations...)])

       #forces = mbsCode[:equations].args[4].args[1].args
       #@show forces
       #dump(forces)
       #jointForces = :( $(jointForces1...), $(jointForces6...) )
       # @show jointForces
       #push!(jointForces, jointForces1)
       #push!(jointForces, jointForces6)      
       #mbsCode[:equations].args[length(mbs_equations)].args[1].args = jointForces
       #@show mbsCode


    elseif method == :ComputeJointAccelerations
        mbsCode = Model(_id = rand(Int),
                        _qdd = Var(start = zeros(ndofTotal)),
                        equations = :[_mbs1 = Modia3D.initJoints!(_id, instantiatedModel, $ndofTotal, time)
                                      _mbs2 = Modia3D.setJointStates1!(_mbs1, $(jointStates1...))
                                      _qdd = Modia3D.getJointResiduals_method3!(_mbs2, $(jointForces1...))
                                      $(jointAcc1...)
                                     ]
                        )

       #acc = mbsCode[:equations].args[3].args[1].args
       #@show acc
       #dump(acc)
       #mbsCode[:equations].args[3].args[1].args = jointAcc1
       #@show mbsCode

    else
        @error "Error should not occur (method = $method)"
    end
    #error("\n!!! Stop to check code generation")
    return mbsCode | model
end


"""
    getJointInfo!(model, jointInfo, path)

Recursively traverse the OrderedDict `model` and search for key :_jointType in a dictionary
that has key :_constructor and return a vector of tuples [(path_i, _jointType_i)]
A tuple contains the path to a joint (where the path is defined as
quoted expression, for example :(a.b.c)), and a tuple _jointInfo, for example :Revolute.
"""
function getJointInfo!(model, jointInfo, path)::Nothing
    if haskey(model, :_constructor)
        constructor = model[:_constructor]
        if typeof(constructor) <: OrderedDict && haskey(constructor, :_jointType)
            push!(jointInfo, (path=path, jointType=constructor[:_jointType]))
            return
        end
    end

    for (key,value) in model
        if typeof(value) <: OrderedDict
            getJointInfo!(value, jointInfo, appendSymbol(path, key))
        end
    end
    return nothing
end
