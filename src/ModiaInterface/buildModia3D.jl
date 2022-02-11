using OrderedCollections

appendSymbol(path::Nothing, name::Symbol) = name
appendSymbol(path         , name::Symbol) = :( $path.$name )

derSymbol(path::Nothing, name::Symbol) = :(der($name))
derSymbol(path         , name::Symbol) = :(der($path.$name))

nextMbsName(mbsi,i) = (Symbol("_mbs"*string(i+1)), mbsi, i+1)


"""
    buildModia3D(model; method = :ComputeJointForces)

Generate and return the following type of code:

```
model | Model(_id = rand(Int),
              equations = :[...])
```

# Arguments

- `model`: Modia model
- `method`: Code generation method. Possible values: `:ComputeJointForces`,`:ComputeJointAccelerations`.
"""
function buildModia3D(model; method = :ComputeJointForces)   # :ComputeJointAccelerations, :ComputeJointAccelerationsOn
    @assert(method == :ComputeJointForces)
    jointInfo = []
    getJointInfo!(model, jointInfo, nothing)

    if length(jointInfo) == 0
        ndofTotal = 0    # what to do???
        args = nothing
        error("No joints present ->  so no states")
    end

    @show method

    #=
                  _mbs1 = Modia3D.initJoints!(_id, instantiatedModel, $ndofTotal, time)
                  _mbs2a = Modia3D.setJointStates1!(         _mbs1 , $(jointStates1...))
                  _mbs2b = Modia3D.setJointStates6!(         _mbs2a, $(jointStates6...))
                  _mbs2c = Modia3D.setJointStates6_isrot123!(_mbs2b, $(jointStates6_isrot123...))

       method==2: # f := M(q)*qdd + h(q,qd)
                  _mbs3a = Modia3D.setJointAccelerations1!(_mbs2c, $(jointAcc1...))
                  _mbs3b = Modia3D.setJointAccelerations6!(_mbs2c, $(jointAcc6...))
                  $(jointForces...) = implicitDependency(Modia3D.getJointResiduals_method2!(_mbs3b, _leq_mode), $(jointAcc1...), $(jointAcc6...))

       method==3: # qdd := M(q)\(h(q,qd)-f)
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
    jointStates1 = Expr[]
    jointForces1 = []
    jointAcc1    = []

    jointStates6 = Expr[]
    jointForces6 = []
    jointAcc6    = []
    jointStates6_isrot123 = Expr[]
    
    i=1
    for joint in jointInfo
        path      = joint.path
        jointType = joint.jointType

        if jointType == :Revolute || jointType == :RevoluteWithFlange
            ndofTotal += 1
            push!(jointStates1, appendSymbol(path, :phi))
            push!(jointStates1, appendSymbol(path, :w))
            if jointType == :RevoluteWithFlange
                push!(jointForces1, appendSymbol(appendSymbol(path, :flange), :tau))
            else
                push!(jointForces1, 0.0 )
            end
            if method == :ComputeJointForces
                push!(jointAcc1, derSymbol(path, :w))
            elseif method == :ComputeJointAccelerations
                der_w = derSymbol(path, :w)
                push!(jointAcc1, :( $der_w = _qdd[$i]) )
                i += 1
            end


        elseif jointType == :Prismatic || jointType == :PrismaticWithFlange
            ndofTotal += 1
            push!(jointStates1, appendSymbol(path, :s))
            push!(jointStates1, appendSymbol(path, :v))
            if jointType == :PrismaticWithFlange
                push!(jointForces1, appendSymbol(appendSymbol(path, :flange), :f))
            else
                push!(jointForces1, :(0.0))
            end
            if method == :ComputeJointForces
                push!(jointAcc1, derSymbol(path, :v))
            elseif method == :ComputeJointAccelerations
                der_v = derSymbol(path, :v)
                push!(jointAcc1, :( $der_v = _qdd[$i]) )
                i += 1
            end

        elseif jointType == :FreeMotion
            ndofTotal += 6
            push!(jointStates6, appendSymbol(path, :r))
            push!(jointStates6, appendSymbol(path, :rot))
            push!(jointStates6, appendSymbol(path, :v))
            push!(jointStates6, appendSymbol(path, :w))
            push!(jointStates6_isrot123, appendSymbol(path, :isrot123))
            push!(jointForces6, :(0.0))
            push!(jointForces6, :(0.0))

            if method == :ComputeJointForces
                push!(jointAcc6, derSymbol(path, :v))
                push!(jointAcc6, derSymbol(path, :w))
            elseif method == :ComputeJointAccelerations
                der_v = derSymbol(path, :v)
                der_w = derSymbol(path, :w)
                push!(jointAcc6, :( $der_v = _qdd[$i]) )
                push!(jointAcc6, :( $der_v = _qdd[$i+1]) )
                i += 2
            end

        else
            error("\nJoint type $jointType in submodel with path $path is not known.")
        end
    end

    i=1
    mbsi = :_mbs1
    mbs_equations = [ :($mbsi = Modia3D.initJoints!(_id, instantiatedModel, $ndofTotal, time)) ]
    
    if length(jointStates1) > 0
        (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
        push!(mbs_equations, :( $mbsi = Modia3D.setJointStates1!($mbsi_old, $(jointStates1...)) ))
    end
    if length(jointStates6) > 0
        (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
        push!(mbs_equations, :( $mbsi = Modia3D.setJointStates6!($mbsi_old, $(jointStates6...)) ))
        (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
        push!(mbs_equations, :( $mbsi = Modia3D.setJointStates6_isrot123!($mbsi_old, $(jointStates6_isrot123...)) ))
    end


    if method == :ComputeJointForces
        if length(jointAcc1) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
            push!(mbs_equations, :( $mbsi = Modia3D.setJointAccelerations1!($mbsi_old, $(jointAcc1...)) ))
        end
        if length(jointAcc6) > 0
            (mbsi, mbsi_old, i) = nextMbsName(mbsi, i)
            push!(mbs_equations, :( $mbsi = Modia3D.setJointAccelerations6!($mbsi_old, $(jointAcc6...)) ))
        end
        #push!(mbs_equations, :( (dummy,) = implicitDependency(Modia3D.getJointResiduals_method2!($mbsi, _leq_mode), $(jointAcc1...), $(jointAcc6...)) ))
        push!(mbs_equations, :( ($(jointForces1...), $(jointForces6...)) = implicitDependency(Modia3D.getJointResiduals_method2!($mbsi, _leq_mode), $(jointAcc1...), $(jointAcc6...)) ))

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

Recursively traverse the OrderedDict `model` and search for key :_joint in a dictionary
that has key :_constructor and return a vector of tuples [(path_i, _jointInfo_i)]
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
