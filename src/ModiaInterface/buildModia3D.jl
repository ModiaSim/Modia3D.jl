using OrderedCollections

appendSymbol(path::Nothing, name::Symbol) = name
appendSymbol(path         , name::Symbol) = :( $path.$name )

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
function buildModia3D(model)    #(model; path="")
    jointInfo = []
    getJointInfo!(model, jointInfo, nothing)

    if length(jointInfo) == 0
        ndofTotal = 0    # what to do???
        args = nothing
        error("No joints present ->  so no states")
    else
        ndofTotal = sum(j.ndof for j in jointInfo)
        args = [appendSymbol(e.path, :variables) for e in jointInfo]
    end

    code = Expr[]
    i = 1
    for e in jointInfo
        if e.ndof == 1
            push!(code, :( $(e.path).qdd = _qdd[$i] ))
        else
            iend = i + e.ndof - 1
            push!(code, :( $(e.path).qdd = _qdd[$i:$iend] ))
        end
        i += e.ndof
    end

    mbsCode = Model(_id = rand(Int),
                    _qdd = Var(start = zeros(ndofTotal)),
                    mbs_equations = :[jointVariablesHaveValues = setModiaJointVariables!(_id, _leq_mode, instantiatedModel, time, $(args...))
                                      0 = multibodyResiduals!(_id, _leq_mode, instantiatedModel, time, jointVariablesHaveValues, _qdd)
                                      $(code...)])

                    #mbs_equations = :[zeros($ndofTotal) = multibodyResiduals(_id, _leq_mode, instantiatedModel, time, _qdd, $(args...))
                    #                  $(code...)])

    return mbsCode | model
end


"""
    getJointInfo!(model, jointInfo, path)

Recursively traverse the OrderedDict `model` and return a vector of tuples.
A tuple contains the path to a joint (where the path is defined as
quoted expression, for example :(a.b.c)), and the length of the `qdd` vector.
"""
function getJointInfo!(model, jointInfo, path)::Nothing
    if haskey(model, :_constructor)
        constructor = model[:_constructor]
        if typeof(constructor) <: OrderedDict && haskey(constructor, :ndof)
            ndof = constructor[:ndof]
            push!(jointInfo, (path=path, ndof=ndof))
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
