# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Basics (Modia3D/Basics/_module.jl)
#


"""
    component = Modia3D.getPathComponent(rootComponent, path::String)

Return the component defined by rootComponent and path.

"""
function getPathComponent(rootComponent, path::String)
    component = rootComponent
    lenPath = length(path)
    if lenPath == 0
        @goto PathInvalid
    end
    i = 1
    while true
        j = findnext(".", path, i)
        key = isnothing(j) ? Symbol(path[i:end]) : Symbol(path[i:j[1]-1])
        if !isdefined(component, key) # hasfield(typeof(component), key)
            @goto PathInvalid
        end
        component = getfield(component, key)
        if isnothing(j)
            return component
        end
        i=j[1]+1
        if i > lenPath
            @goto PathInvalid
        end
    end

    @label PathInvalid
    T = typeof(rootComponent)
    error("getPathComponent(rootComponent::$T, \"$path\"): path is wrong.\n",
          "Allowed symbols on the last correct level:\n",
          fieldnames(typeof(component)))
end
