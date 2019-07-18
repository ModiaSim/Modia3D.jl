# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Basics (Modia3D/Basics/_module.jl)
#


"""
    path = PTP_path(names;
                    positions = [zeros(size(names,1))'; ones(size(names,1))'],
                    startTime=0.0,
                    v_max = ones(size(names,1)),
                    a_max = ones(size(names,2))))

Generate a new path object.
"""
mutable struct PTP_path
   names::Vector{AbstractString}
   startTime::Float64
   v_max::Vector{Float64}
   a_max::Vector{Float64}
   positions::Matrix{Float64}

   delta::Matrix{Float64}
   hasPath::Vector{Bool}
   sd_max::Vector{Float64}
   sdd_max::Vector{Float64}
   Ta1::Vector{Float64}
   Ta2::Vector{Float64}
   noWphase::Vector{Bool}
   Tv::Vector{Float64}
   Te::Vector{Float64}
   Ta1s::Vector{Float64}
   Ta2s::Vector{Float64}
   Tvs::Vector{Float64}
   Tes::Vector{Float64}
   sd_max2::Vector{Float64}
   s1::Vector{Float64}
   s2::Vector{Float64}
   s3::Vector{Float64}
   Tend::Float64
   posTemp::Vector{Float64}   # Temporary storage that can be used by the functions operation on PTP_path

   function PTP_path(names::AbstractVector;
                     positions::Matrix{Float64} = [zeros(size(names,1))'; ones(size(names,1))'],
                     startTime=0.0,
                     v_max = ones(size(names,1)),
                     a_max = ones(size(names,2)))

        #@assert(size(positions,1) > 1)
        #@assert(size(positions,2) == Base.length(names))
        #@assert(Base.length(v_max) == Base.length(names))
        #@assert(Base.length(a_max) == Base.length(names))
        np    = Base.length(names)
        npath = size(positions,1) - 1   # number of path points

        for i in eachindex(v_max)
            @assert(v_max[i] > 0.0)
            @assert(a_max[i] > 0.0)
        end

        delta    = zeros(npath, np)
        hasPath  = fill(false,npath)
        sd_max   = zeros(npath)
        sdd_max  = zeros(npath)
        Ta1      = zeros(npath)
        Ta2      = zeros(npath)
        noWphase = fill(false,npath)
        Tv       = zeros(npath)
        Te       = zeros(npath)
        Ta1s     = zeros(npath)
        Ta2s     = zeros(npath)
        Tvs      = zeros(npath)
        Tes      = zeros(npath)
        sd_max2  = zeros(npath)
        s1       = zeros(npath)
        s2       = zeros(npath)
        s3       = zeros(npath)
        aux1     = zeros(np)
        aux2     = zeros(np)

        for i in 1:npath
            delta[i,:] = positions[i+1,:] - positions[i,:]

            for j in 1:np
                aux1[j] = delta[i,j]/v_max[j]
                aux2[j] = delta[i,j]/a_max[j]
            end

            sd_max_inv  = maximum(abs.(aux1))
            sdd_max_inv = maximum(abs.(aux2))

            small = 1000*eps()
            if sd_max_inv <= small || sdd_max_inv <= small
                hasPath[i] = false
            else
                hasPath[i] = true
                sd_max[i]  = 1/sd_max_inv
                sdd_max[i] = 1/sdd_max_inv

                Ta1[i] = sqrt(1/sdd_max[i])
                Ta2[i] = sd_max[i]/sdd_max[i]
                noWphase[i] = Ta2[i] >= Ta1[i]

                Tv[i]   = noWphase[i] ? Ta1[i] : 1/sd_max[i]
                Te[i]   = noWphase[i] ? Ta1[i] + Ta1[i] : Tv[i] + Ta2[i]

                Tbegin  = i==1 ? startTime : Tes[i-1]
                Ta1s[i] = Ta1[i] + Tbegin
                Ta2s[i] = Ta2[i] + Tbegin
                Tvs[i]  = Tv[i]  + Tbegin
                Tes[i]  = Te[i]  + Tbegin
                sd_max2[i] = sdd_max[i]*Ta1[i]
                s1[i] = sdd_max[i]*(noWphase[i] ? Ta1[i]*Ta1[i] : Ta2[i]*Ta2[i])/2
                s2[i] = s1[i] + (noWphase[i] ? sd_max2[i]*(Te[i] - Ta1[i]) - (sdd_max[i]/2)*(Te[i] - Ta1[i])^2 : sd_max[i]*(Tv[i] - Ta2[i]))
                s3[i] = s2[i] + sd_max[i]*(Te[i] - Tv[i]) - (sdd_max[i]/2)*(Te[i] - Tv[i])^2
            end
        end

        Tend = Tes[end]

        new(names, startTime, v_max, a_max, positions, delta,
            hasPath, sd_max, sdd_max, Ta1, Ta2, noWphase, Tv, Te, Ta1s, Ta2s, Tvs, Tes, sd_max2, s1, s2, s3, Tend, zeros(np))
    end
end


"""
    getPosition!(path, time, position)

Given a `path::PTP_path` and a time instant `time`, return the actual
position at time `time` in vector `position`.
"""
function getPosition!(path::PTP_path, time::Number, position::Vector{Float64})
    time = ustrip(time)
    @assert(length(position) == size(path.positions,2))
    npath = length(path.hasPath)
    np    = length(position)

    # Search correct time interval
    i = 0
    if time <= path.startTime
        i = 1
        s = 0

    else
        while i < npath
            i = i+1
            if time <= path.Tes[i]
               break
            end
        end
        if time >= path.Tes[end]
            i = npath
            s = path.noWphase[i] ? path.s2[end] : path.s3[end]
            #println("... time=$time i=$i s=$s qbegin=", path.positions[i,1], ", qdelta = ", path.delta[i,1])
        else
            Tbegin = i==1 ? path.startTime : path.Tes[i-1]
            if path.noWphase[i]
                if time < path.Ta1s[i]
                    s = (path.sdd_max[i]/2)*(time - Tbegin)^2
                elseif time < path.Tes[i]
                    s = path.s1[i] + path.sd_max2[i]*(time - path.Ta1s[i]) - (path.sdd_max[i]/2)*(time - path.Ta1s[i])^2
                else
                    s = path.s2[i]
                end
            elseif time < path.Ta2s[i]
                s = (path.sdd_max[i]/2)*(time - Tbegin)^2
            elseif time < path.Tvs[i]
                s = path.s1[i] + path.sd_max[i]*(time - path.Ta2s[i])
            elseif time < path.Tes[i]
                s = path.s2[i] + path.sd_max[i]*(time - path.Tvs[i]) - (path.sdd_max[i]/2)*(time - path.Tvs[i])*(time - path.Tvs[i])
            else
                s = path.s3[i]
            end
        end
    end

    for j in 1:np
        # println("... i = $i, j= $j, np = $np, s=$s, positions[i,1]=",path.positions[i,1])
        position[j]= path.positions[i,j] + path.delta[i,j]*s
    end
end


"""
    pos = getPosition(path, index, time)

Given a `path::PTP_path`, the `index` of a signal, and a time instant `time`, return the actual
position at time `time`.
"""
function getPosition(path::PTP_path, index, time::Number)
    getPosition!(path, time, path.posTemp)
    return path.posTemp[index]
end



"""
    index = getIndex(path, name)

Return the index of `name` in `path` or trigger an error, if not present.
"""
function getIndex(path, name)
    index = findfirst(x -> x==name, path.names)
    if typeof(index) == Nothing
        error("getIndex(path,name): \"", name, "\" not in path")
    end
    return index
end


"""
    plotPath(path;
             names=path.names, heading="PTP plots",
             tend=1.1*path.Tend, figure=1, ntime=101)

Given a `path::PTP_path`, plot the path over `time` up to `tend` for all points
identified by the vector or tuple `names` to figure `figure`
using `ntime` time points.
"""
function plotPath(path::PTP_path; names=path.names, heading="PTP plots", figure=1, ntime=101, tend = 1.1*path.Tend)
    time = range(0u"s",(tend)u"s",length=ntime)
    indices = indexin(names, path.names)
    names2  = deepcopy(names)
    for i in eachindex(indices)
        if typeof(i) == Nothing
            @warn "plotPath(path, ...): \""*names[i]*"\" is ignored, because not in path"
            deleteat!(indices,i)
            deleteat!(names2,i)
        end
    end

    np   = length(indices)
    q    = zeros(length(time), np)
    qt   = zeros(length(path.names))
    for i in eachindex(time)
        getPosition!(path, time[i], qt)
        q[i,:] = qt[indices]
    end

    series = Dict{AbstractString,Any}()
    series["time"] = time
    for i in eachindex(names2)
        series[names2[i]] = q[:,i]
    end

    ModiaMath.plot(series, Tuple(names2), heading=heading, figure=figure)
end
