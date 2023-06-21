### ------------ compute contact forces/torques --------------------------------
# functions setComputationFlag(), selectContactPairsWithEvent!(),
# selectContactPairsNoEvent!() and getDistances!() are called to compute distances
# between objs with mpr algorithm, and use them as zero-crossing functions
# (see file ...\ContactDetectionMPR\handler.jl)
# after computing all distances between colliding shapes response calculations are done
function computeContactForcesAndTorques(sim::Modia.InstantiatedModel, scene, world, time, file)
    if typeof(scene.options.contactDetection) <: Modia3D.ContactDetectionMPR_handler
        ch::Modia3D.ContactDetectionMPR_handler = scene.options.contactDetection
        # Compute signed distances of all contact shapes during zero-crossing computation
            #try
        setComputationFlag(ch)
        if Modia.isEvent(sim)             # with Event
            # handle event
            TimerOutputs.@timeit sim.timer "Modia3D_1 selectContactPairsWithEvent!"  selectContactPairsWithEvent!(sim, scene, ch, world)
        elseif Modia.isZeroCrossing(sim)  # no Event
            # compute zero crossings
            TimerOutputs.@timeit sim.timer "Modia3D_1 selectContactPairsNoEvent!" selectContactPairsNoEvent!(sim, scene, ch, world)
        else
            # compute contact distances
            TimerOutputs.@timeit sim.timer "Modia3D_1 getDistances!"  getDistances!(scene, ch, world)
        end
        TimerOutputs.@timeit sim.timer "Modia3D_1 dealWithContacts!"  dealWithContacts!(sim, scene, ch, world, time, file)
        # dealWithContacts!(sim, scene, ch, world, time, file)
    #catch e
        #println("Bug in Modia3D/src/Composition/dynamicCollision.jl: error curing collision detection at time = ", time)
    #end
    else
        error("No contact detection handler is defined")
    end
    return nothing
end

# each shape pair in contact is stored in ch.contactDict
# if there is an event simulation status is updated and for response calculation
# a contact material is defined
# contact and/or support points are visualized
# only if enableContactDetection is true responseCalculation for force and torque computation is called
# (see file ...\Composition\responseCalculation\elasticCollisionResponse.jl)
# further, at an event simulation status is updated, contact material is replaced
# and the actual contactDict is stored
function dealWithContacts!(sim::Modia.InstantiatedModel{F, T}, scene::Scene{F}, ch::Composition.ContactDetectionMPR_handler{M,F}, world::Composition.Object3D{F}, time::Float64, file::Nothing)::Nothing where {F <: Modia3D.VarFloatType, T, M}

    simh::Modia.EventHandler{F,T} = sim.eventHandler
    f1::SVector{3,F}=Modia3D.ZeroVector3D(F)
    f2::SVector{3,F}=Modia3D.ZeroVector3D(F)
    t1::SVector{3,F}=Modia3D.ZeroVector3D(F)
    t2::SVector{3,F}=Modia3D.ZeroVector3D(F)

    for (pairID::Int64, pair::ContactPair{F}) in ch.contactDict
        obj1 = pair.obj1
        obj2= pair.obj2
        rContact= (pair.contactPoint1 + pair.contactPoint2)/F(2.0)
        contactNormal = pair.contactNormal
        if Modia.isEvent(sim)
            # println("$(sim.time): ", obj1.path, " ", obj2.path)
            getMaterialContactStart(scene, ch, simh, pair, pairID, obj1, obj2, rContact, contactNormal)
            visualizeContactAndSupportPoints(ch, world)
        end
        if scene.options.enableContactDetection
            if pair.pairKind == Modia3D.NoContactPairKind
                noContactPairMaterial::Shapes.NoContactPairMaterial = pair.contactPairMaterial
                (f1,f2,t1,t2) = responseCalculation(noContactPairMaterial, obj1, obj2,
                            rContact, contactNormal,
                            pair.distanceWithHysteresis, time, file, sim)
            elseif pair.pairKind == Modia3D.ElasticContactPairKind
                elasticContactPairMaterial::Composition.ElasticContactPairResponseMaterial = pair.contactPairMaterial
                (f1,f2,t1,t2) = responseCalculation(elasticContactPairMaterial, obj1, obj2,
                    rContact, contactNormal,
                    pair.distanceWithHysteresis, time, file, sim)
            elseif pair.pairKind == Modia3D.ObserverContactPairKind
                observerContactPairMaterial::Shapes.ObserverContactPairMaterial = pair.contactPairMaterial
                (f1,f2,t1,t2) = responseCalculation(observerContactPairMaterial, obj1, obj2,
                rContact, contactNormal,
                pair.distanceWithHysteresis, time, file, sim)
            elseif pair.pairKind == Modia3D.ImpulseContactPairKind
                impulseContactPairMaterial::Shapes.ImpulseContactPairMaterial = pair.contactPairMaterial
                (f1,f2,t1,t2) = responseCalculation(impulseContactPairMaterial, obj1, obj2,
                rContact, contactNormal,
                pair.distanceWithHysteresis, time, file, sim)
            elseif pair.pairKind == Modia3D.WheelRailContactPairKind
                wheelRailContactPairMaterial::Shapes.WheelRailContactPairMaterial = pair.contactPairMaterial
                (f1,f2,t1,t2) = responseCalculation(wheelRailContactPairMaterial, obj1, obj2,
                rContact, contactNormal,
                pair.distanceWithHysteresis, time, file, sim)
            else
                error("not supported contact pair material")
            end

            # Transform forces/torques in local part frames
            obj1.f += obj1.R_abs*f1
            obj1.t += obj1.R_abs*t1
            obj2.f += obj2.R_abs*f2
            obj2.t += obj2.R_abs*t2
        end
    end

    if Modia.isEvent(sim)
        deleteMaterialLastContactDictContactEnd(scene, ch, simh)
    end
    return nothing
end


# mainly its for assigning contact materials.
# therefore, lastContactDict stores information of shape pairs in contact at last step
# if the actual shape pair was in contact at the last step as well, the already assigned contact material is taken
# otherwise, contact material between these two shapes is choosen and simulation status is set
function getMaterialContactStart(scene, ch, simh, pair, pairID, obj1, obj2, rContact, contactNormal)
    if haskey(ch.lastContactDict, pairID)
        # use material (reference) from previous event
        if scene.options.enableContactDetection
            pair.contactPairMaterial = ch.lastContactDict[pairID].contactPairMaterial    # improve later (should avoid to inquire pairID twice)
            pair.pairKind = ch.lastContactDict[pairID].pairKind
        end
    else
        # determine contact pair material
        if scene.options.enableContactDetection
            (pair.contactPairMaterial, pair.pairKind) = contactStart(obj1, obj2, rContact, contactNormal,
                scene.options.elasticContactReductionFactor,
                scene.options.maximumContactDamping)
        end
        simh.restart = max(simh.restart, Modia.Restart)
        simh.newEventIteration = false
        if scene.options.enableContactDetection
            logEvents(simh, pair, obj1, obj2, rContact, contactNormal)
        end
        return nothing
end; end

# important feature is stored at an event
function logEvents(simh, pair, obj1, obj2, rContact, contactNormal)
    if simh.logEvents
        name1 = Modia3D.fullName(obj1)
        name2 = Modia3D.fullName(obj2)
        n     = contactNormal
        println("        distance(", name1, ",", name2, ") = ", pair.distanceWithHysteresis, " became <= 0")

        @inbounds println("            contact normal = [", round(n[1], sigdigits=3), ", ", round(n[2], sigdigits=3), ", ", round(n[3], sigdigits=3), "], contact position = [", round(rContact[1], sigdigits=3), ", ", round(rContact[2], sigdigits=3), ", ", round(rContact[3], sigdigits=3),"], c_res = ", round(pair.contactPairMaterial.c_res, sigdigits=3) , " d_res = ", round(pair.contactPairMaterial.d_res, sigdigits=3))
    end
    return nothing
end

# each contact shape pair which was in lastContactDict but isn't in actual contactDict
# (= shape pair was in contact at last step but isn't at the actual step)
# contact material is "deleted", it is set to nothing
# and simulation status is set.
# lastContactDict is deleted and filled with feature of the actual contactDict
function deleteMaterialLastContactDictContactEnd(scene, ch, simh)
    for (pairID, pair) in ch.lastContactDict
        if !haskey(ch.contactDict, pairID)
            if scene.options.enableContactDetection
                contactEnd(pair.contactPairMaterial, pair.obj1, pair.obj2)
            end
            simh.restart = max(simh.restart, Modia.Restart)
            simh.newEventIteration = false
            if !simh.logEvents    # Modia.isLogEvents(simh.logger)
                break
            end
            name1 = Modia3D.fullName(pair.obj1)
            name2 = Modia3D.fullName(pair.obj2)
            println("        distance(", name1, ",", name2, ") = ", pair.distanceWithHysteresis, " became > 0")
        end
    end

    # delete lastContactDict and save contactDict in lastContactDict
    empty!(ch.lastContactDict)
    for (pairID, pair) in ch.contactDict
        push!(ch.lastContactDict, pairID => pair)
    end
    return nothing
end


### ------- visualizing contact and support points -----------------------------
# this function visualizes contact and support points if visualizeContactPoints and/or
# visualizeSupportPoints is true
# if a contact point and/or support point is nothing the point is transparent and therefore invisible
# otherwise its visualized as a sphere
function visualizeContactAndSupportPoints(ch, world::Composition.Object3D{F}) where F <: Modia3D.VarFloatType
    contactDictCollect = collect(ch.contactDict)
    lengthDict = length(ch.contactDict)

    if ch.visualizeContactPoints
        maxLength = length(world.contactVisuObj1)
        @inbounds for i=1:maxLength
            if i <= lengthDict
                transparency = 0.0
                point1 = contactDictCollect[i][2].contactPoint1
                point2 = contactDictCollect[i][2].contactPoint2
            else
                transparency = 1.0
                point1 = Modia3D.ZeroVector3D(F)
                point2 = Modia3D.ZeroVector3D(F)
            end
            setVisualizationContactProperties!(world.contactVisuObj1[i], transparency, point1)
            setVisualizationContactProperties!(world.contactVisuObj2[i], transparency, point2)
        end
        if lengthDict > maxLength
            nVisualContSupPoints = lengthDict
            printWarnContSupPoints(nVisualContSupPoints)
        end
    end

    if ch.visualizeSupportPoints
        maxLength = length(world.supportVisuObj1A)
        @inbounds for i=1:maxLength
            if i <= lengthDict && contactDictCollect[i][2].supportPointsDefined
                transparency = 0.0
                support1A = contactDictCollect[i][2].support1A
                support2A = contactDictCollect[i][2].support2A
                support3A = contactDictCollect[i][2].support3A
                support1B = contactDictCollect[i][2].support1B
                support2B = contactDictCollect[i][2].support2B
                support3B = contactDictCollect[i][2].support3B
            else
                transparency = 1.0
                support1A = Modia3D.ZeroVector3D(F)
                support2A = Modia3D.ZeroVector3D(F)
                support3A = Modia3D.ZeroVector3D(F)
                support1B = Modia3D.ZeroVector3D(F)
                support2B = Modia3D.ZeroVector3D(F)
                support3B = Modia3D.ZeroVector3D(F)
            end
            setVisualizationContactProperties!(world.supportVisuObj1A[i], transparency, support1A)
            setVisualizationContactProperties!(world.supportVisuObj2A[i], transparency, support2A)
            setVisualizationContactProperties!(world.supportVisuObj3A[i], transparency, support3A)
            setVisualizationContactProperties!(world.supportVisuObj1B[i], transparency, support1B)
            setVisualizationContactProperties!(world.supportVisuObj2B[i], transparency, support2B)
            setVisualizationContactProperties!(world.supportVisuObj3B[i], transparency, support3B)
        end
        if lengthDict > maxLength
            nVisualContSupPoints = lengthDict
            printWarnContSupPoints(nVisualContSupPoints)
        end
    end
    return nothing
end

printWarnContSupPoints(nVisualContSupPoints) = @warn("If all contact points and/or support points should be visualized please set nVisualContSupPoints = $nVisualContSupPoints in Scene.")

# the sphere is visible and its absolute position is updated (this point was computed by the mpr algorithm)
function setVisualizationContactProperties!(obj::Composition.Object3D{F}, transparency::Float64, point::SVector{3,F}) where F <: Modia3D.VarFloatType
    obj.r_abs = point
    obj.feature.visualMaterial.transparency = transparency
    return nothing
end


function getElasticContactPair(sim::Modia.InstantiatedModel{F, T}, scene::Scene{F}, obj1::String, obj2::String) where {F <: Modia3D.VarFloatType, T}
    ch::Modia3D.ContactDetectionMPR_handler = scene.options.contactDetection
    obj1::Composition.Object3D{F} = getComponent(sim, obj1)
    obj2::Composition.Object3D{F} = getComponent(sim, obj2)

    for (pairID::Int64, pair::ContactPair{F}) in ch.contactDict
        obj1pair = pair.obj1
        obj2pair = pair.obj2

        if scene.options.enableContactDetection
            if pair.pairKind == Modia3D.ElasticContactPairKind
                if (obj1 == obj1pair && obj2 == obj2pair) || (obj2 == obj1pair && obj1 == obj2pair)
                    return pair
                end
            end
        end
    end

    return nothing
end

function getComponent(sim::Modia.InstantiatedModel{F, T}, path::String) where {F <: Modia3D.VarFloatType, T <: AbstractFloat}
    obj::Composition.Object3D{F} = Modia.getValue(sim, path)
    return obj
end
