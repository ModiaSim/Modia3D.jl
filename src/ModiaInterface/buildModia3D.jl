using OrderedCollections

appendSymbol(path::Nothing, name::Symbol) = name
appendSymbol(path         , name::Symbol) = :( $path.$name )

derSymbol(path::Nothing, name::Symbol) = :(der($name))
derSymbol(path         , name::Symbol) = :(der($path.$name))

"""
    buildModia3D(model)

Generate the following type of code:

```
model | Model(_id = rand(Int),
              start = Map(_qdd = zeros(1)),
              mbs_equations = :[
                 zeros(1) = multibodyResiduals(_id, _leq_mode, instantiatedModel, _qdd, rev.variables)
                 rev.qdd = _qdd[1]])
```
"""
function buildModia3D(model; method::Int=2) 
    @assert(method>=2 && method <=3)
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
                  _mbs2 = Modia3D.setJointStates1!(_mbs1, $(jointStates1...))  
                  
       method==2: # f := M(q)*qdd + h(q,qd)
                  _mbs3 = Modia3D.setJointAccelerations1!(_mbs2, $(jointAcc1...))
                  $(jointForces1...) = implicitDependency(Modia3D.getJointResiduals_method2!(_mbs3, _leq_mode), $(jointAcc1...))
      
       method==3: # qdd := M(q)\(h(q,qd)-f)                          
                  _qdd  = Modia3D.getJointResiduals_method3!(_mbs2, $(jointForces1...))
                  _qdd[1] = ...
                  _qdd[2] = ...      
   =#                   
 
    ndofTotal = 0
    jointStates1     = Expr[]
    jointForces1     = []
    zeroJointForces1 = Expr[]
    jointAcc1        = []
    zerosTuple       = Expr(:tuple)
    i = 1
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
            if method == 3
                der_w = derSymbol(path, :w)
                push!(jointAcc1, :( $der_w = _qdd[$i]) )
            else
                push!(jointAcc1, derSymbol(path, :w))              
            end    
            i += 1

        elseif jointType == :Prismatic || jointType == :PrismaticWithFlange
            ndofTotal += 1
            push!(jointStates1, appendSymbol(path, :s))
            push!(jointStates1, appendSymbol(path, :v))
            if jointType == :PrismaticWithFlange
                push!(jointForces1, appendSymbol(appendSymbol(path, :flange), :f))
            else
                push!(jointForces1, :(0.0))
            end
            if method == 3
                der_v = derSymbol(path, :v)
                push!(jointAcc1, :( $der_v = _qdd[$i]) )
            else
                push!(jointAcc1, derSymbol(path, :v))            
            end
            i += 1

        else
            error("Joint type $jointType in submodel with path $path is not known.")
        end
        push!(zerosTuple.args, 0)
    end

    if method==2 
        mbsCode = Model(_id = rand(Int),
                        equations = :[_mbs1 = Modia3D.initJoints!(_id, instantiatedModel, $ndofTotal, time)
                                      _mbs2 = Modia3D.setJointStates1!(_mbs1, $(jointStates1...))  
                                      _mbs3 = Modia3D.setJointAccelerations1!(_mbs2, $(jointAcc1...))
                                      (dummy,) = implicitDependency(Modia3D.getJointResiduals_method2!(_mbs3, _leq_mode), $(jointAcc1...))
                                     ]
                        )
                        
       #forces = mbsCode[:equations].args[4].args[1].args
       #@show forces
       #dump(forces)
       mbsCode[:equations].args[4].args[1].args = jointForces1
       #@show mbsCode
       
    elseif method==3 
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

    for (key,value) in model # zip(keys(model), model)
        if typeof(value) <: OrderedDict
            getJointInfo!(value, jointInfo, appendSymbol(path, key))
        end
    end
    return nothing
end
