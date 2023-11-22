﻿# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


function build_tree!(scene::Scene{F}, world::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    tree                 = scene.tree
    stack                = scene.stack
    allCollisionElements = scene.allCollisionElements
    empty!(tree)
    empty!(stack)
    empty!(scene.userDefinedObject3Ds)
    empty!(scene.visualObject3Ds)
    empty!(scene.pureResultObject3Ds)
    empty!(allCollisionElements)

    fillVisuElements!(scene, world, world)
    append!(stack, world.children)
    while length(stack) > 0
        obj = pop!(stack)
        fillVisuElements!(scene, obj, world)
        if canCollide(obj)
            push!(allCollisionElements, obj)
        end
        push!(tree, obj)
        # Visit children of frame
        append!(stack, obj.children)
    end
    if length(allCollisionElements) > 1
        scene.collide = true
    else
        scene.collide = false
    end
    return nothing
end

insert_and_dedup!(v::Vector, x) = (splice!(v, searchsorted(v,x), x); v)

function addIndicesOfCutJointsToSuperObj(scene::Scene{F})::Nothing where F <: Modia3D.VarFloatType
    tmp = collect(values(scene.noCPairsHelp))
    for i=1:length(tmp)
        if length(tmp[i]) == 2
            insert_and_dedup!(scene.noCPairs[minimum(tmp[i])], maximum(tmp[i]))
        else
            error("...from addIndicesOfCutJointsToSuperObj: problems with amount of cut joints")
        end
    end
    return nothing
end


function createAABB_noCPairs(scene::Scene{F}, superObjsRow::SuperObjsRow{F})::Nothing where F <: Modia3D.VarFloatType
    if length(superObjsRow.superObjCollision.superObj) > 0 && !isempty(superObjsRow.noCPair)
        push!(scene.noCPairs, superObjsRow.noCPair)
    else
        push!(scene.noCPairs, [0])
    end

    if length(superObjsRow.superObjCollision.superObj) > 0
        AABBrow = [Basics.BoundingBox{F}() for i = 1:length(superObjsRow.superObjCollision.superObj)]
        push!(scene.AABB, AABBrow)
    else
        push!(scene.AABB, [])
    end
    return nothing
end


# re-attach obj from obj.parent to newParent
# relative kinematics where obj.parent is defined relative to newParent
function changeParentToRootObj(newParent::Object3D{F}, obj::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    if !isRootObject(newParent)
        error("from changeParentToRootObj: new parent is not a root object!")
    end
    # c = child, o = old parent, n = new parent = root object
    child_r_rel  = obj.r_rel         # o_r_oc
    child_R_rel  = obj.R_rel         # R_co
    parent_r_rel = obj.parent.r_rel  # n_r_no
    parent_R_rel = obj.parent.R_rel  # R_on

    obj.r_rel = parent_r_rel + parent_R_rel' * child_r_rel  # n_r_nc := n_r_no + R_no*o_r_oc
    obj.R_rel = child_R_rel * parent_R_rel                  # R_cn := R_co*R_on

    # Basics.deleteItem(obj.parent.children, obj) # klappt an dieser Stelle nicht
    # Reverse obj, so that newParent is the new parent
    obj.parent = newParent
    push!(newParent.children, obj)
    return nothing
end


# re-attach obj from obj.parent to newParent or newParent.parent
# absolute kinematics
function changeParent(newParent::Object3D{F}, obj::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    if isRootObject(newParent)
        # c = child, n = new parent = root object
        child_r_abs  = obj.r_abs            # 0_r_0c
        child_R_abs  = obj.R_abs            # R_c0
        parent_r_abs_new = newParent.r_abs  # 0_r_0n
        parent_R_abs_new = newParent.R_abs  # R_n0

        obj.r_rel = parent_R_abs_new * (child_r_abs - parent_r_abs_new)  # n_r_nc := R_n0*(0_r_0c - 0_r_0n)
        obj.R_rel = child_R_abs * parent_R_abs_new'                      # R_cn := R_c0*R_0n

        Basics.deleteItem(obj.parent.children, obj)
        obj.parent = newParent
        push!(newParent.children, obj)
    elseif isRootObject(newParent.parent)
        changeParent(newParent.parent, obj)
    else
        error("from changeParent: this should not happen")
    end
    return nothing
end


function getRootObj(obj::Object3D{F}) where F <: Modia3D.VarFloatType
    if isRootObject(obj)
        return obj
    else
        getRootObj(obj.parent)
end; end


function getWorldObj(obj::Object3D{F}) where F <: Modia3D.VarFloatType
    root = getRootObj(obj)
    if isWorld(root)
        return root
    else
        getWorldObj(root.parent)
    end
end

# the indices of super objects, which can't collide, are stored in a list
function fillStackOrBuffer!(scene::Scene{F}, superObj::SuperObjsRow{F}, obj::Object3D{F}, rootSuperObj::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    n_children = length(obj.children)
    help = fill(false, n_children)

    for i = 1:n_children
        child = obj.children[i]
        if isNotWorld(child)
            if isNotFixed(child)
                # joint with at least one dof between obj and child
                push!(scene.buffer, child)
                child.isRootObj = true
                child.computeAcceleration = true
                if !child.canCollide
                    push!(superObj.noCPair, length(scene.buffer))
                end
            else
                if isMovable(child)
                    push!(scene.buffer, child)
                    child.isRootObj = true
                    child.computeAcceleration = true
                else
                    # child is fixed to obj
                    push!(scene.stack, child)
                    assignAccVelo(scene.treeAccVelo, child)
                    if !(obj == rootSuperObj)
                        # re-attach child from obj to rootSuperObj
                        changeParentToRootObj(rootSuperObj, child)
                        help[i] = true
    end; end; end; end; end

    if !isempty(obj.children)
        deleteat!(obj.children,help)
    end
    return nothing
end





# it builds the elements which may collide
# to reduce the amount of collision pairs, some assumptions are made:
#   elements which are rigidly attached, can't collide
#     these elements form together a super object
#   elements which are directly connected with a joint can't collide
#     these elements are excluded from the collision list
function build_superObjs!(scene::Scene{F}, world::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    if !scene.initSuperObj
        stack = scene.stack
        buffer = scene.buffer
        treeAccVelo  = scene.treeAccVelo
        empty!(stack)
        empty!(buffer)
        empty!(treeAccVelo)
        empty!(scene.userDefinedObject3Ds)
        empty!(scene.visualObject3Ds)
        empty!(scene.pureResultObject3Ds)

        world.computeAcceleration = true
        world.isRootObj = true # all objs stored in buffer are root objs
        push!(buffer, world)
        actPos = 1
        nPos   = 1

        hasOneCollisionSuperObj  = false
        hasMoreCollisionSuperObj = false

        while actPos <= nPos
            superObjsRow = SuperObjsRow{F}()
            AABBrow      = Vector{Basics.BoundingBox{F}}[]
            rootSuperObj = buffer[actPos]

            fillVisuElements!(scene, rootSuperObj, world)
            if rootSuperObj != world
                assignAll(scene, superObjsRow, rootSuperObj, world, actPos)
                push!(treeAccVelo, rootSuperObj)
            end
            fillStackOrBuffer!(scene, superObjsRow, rootSuperObj, rootSuperObj)

            if isMovable(rootSuperObj)
                push!(scene.allowedToMove, true)
            else
                push!(scene.allowedToMove, nothing)
            end

            while length(stack) > 0
                frameChild = pop!(stack)
                fillVisuElements!(scene, frameChild, world)
                assignAll(scene, superObjsRow, frameChild, world, actPos)
                fillStackOrBuffer!(scene, superObjsRow, frameChild, rootSuperObj)
            end

            if length(superObjsRow.superObjCollision.superObj) > 0 && hasOneCollisionSuperObj == true
                hasMoreCollisionSuperObj = true
            elseif length(superObjsRow.superObjCollision.superObj) > 0
                hasOneCollisionSuperObj = true
            end

            createAABB_noCPairs(scene, superObjsRow)
            push!(scene.superObjs, superObjsRow)
            nPos = length(buffer)
            actPos += 1
        end
        addIndicesOfCutJointsToSuperObj(scene)
        @assert(length(scene.noCPairs) == length(scene.superObjs))



        #=
        println("first elements of tree")
        println("[")
        for a in scene.treeAccVelo
            println("parent = ", Modia3D.fullName(a.parent))
            println(Modia3D.fullName(a))
            println("[")
            for b in a.children
            println("children = ", Modia3D.fullName(b))
            end
            println("]")
        end
        println("]")
        =#
        #=
        println("superObjRow.superObjCollision.superObj")
        for superObjRow in scene.superObjs
        println("[")
        for a in superObjRow.superObjCollision.superObj
            println(Modia3D.fullName(a))
        end
        println("]")
        println(" ")
        end
        =#

        #=
        println("allowedToMove")
        for a in scene.allowedToMove
            show(a)
            println(" ")
        end
        =#

        #=
        println("superObjRow.superObjMovable.superObj")
        for superObjRow in scene.superObjs
        println("[")
        for a in superObjRow.superObjMovable.superObj
            println(Modia3D.fullName(a))
        end
        println("]")
        println(" ")
        end
        =#
        #=
        println("superObjRow.superObjMass.superObj")
        for superObjRow in scene.superObjs
        println("[")
        for a in superObjRow.superObjMass.superObj
            println(Modia3D.fullName(a))
        end
        println("]")
        println(" ")
        end
        =#


        if hasMoreCollisionSuperObj
            scene.collide = true
        else
            scene.collide = false
        end
        scene.initSuperObj = true
    end
    return nothing
end


function rebuild_superObjs!(scene::Scene{F}, world::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    empty!(scene.stack)
    empty!(scene.buffer)
    empty!(scene.treeAccVelo)
    empty!(scene.allowedToMove)
    empty!(scene.superObjs)
    empty!(scene.noCPairs)
    empty!(scene.noCPairsHelp)
    empty!(scene.AABB)

    stack = scene.stack
    buffer = scene.buffer
    treeAccVelo  = scene.treeAccVelo

    world.computeAcceleration = true
    world.isRootObj = true # all objs stored in buffer are root objs
    push!(buffer, world)
    actPos = 1
    nPos   = 1

    hasOneCollisionSuperObj  = false
    hasMoreCollisionSuperObj = false

    while actPos <= nPos
        superObjsRow = SuperObjsRow{F}()
        AABBrow      = Vector{Basics.BoundingBox{F}}[]
        rootSuperObj = buffer[actPos]

        if rootSuperObj != world
            assignAll(scene, superObjsRow, rootSuperObj, world, actPos)
            push!(treeAccVelo, rootSuperObj)
        end
        fillStackOrBuffer!(scene, superObjsRow, rootSuperObj, rootSuperObj)

        if isMovable(rootSuperObj)
            push!(scene.allowedToMove, true)
        else
            push!(scene.allowedToMove, nothing)
        end

        while length(stack) > 0
            frameChild = pop!(stack)
            assignAll(scene, superObjsRow, frameChild, world, actPos)
            fillStackOrBuffer!(scene, superObjsRow, frameChild, rootSuperObj)
        end

        if length(superObjsRow.superObjCollision.superObj) > 0 && hasOneCollisionSuperObj == true
            hasMoreCollisionSuperObj = true
        elseif length(superObjsRow.superObjCollision.superObj) > 0
            hasOneCollisionSuperObj = true
        end

        createAABB_noCPairs(scene, superObjsRow)
        push!(scene.superObjs, superObjsRow)
        nPos = length(buffer)
        actPos += 1
    end
    addIndicesOfCutJointsToSuperObj(scene)
    @assert(length(scene.noCPairs) == length(scene.superObjs))

    #=
    println("first elements of tree")
    println("[")
    for a in scene.treeAccVelo
      println("parent = ", ModiaMath.fullName(a.parent))
      println(ModiaMath.fullName(a))
      println("[")
      for b in a.children
        println("children = ", ModiaMath.fullName(b))
      end
      println("]")
    end
    println("]")
=#


    if hasMoreCollisionSuperObj
        scene.collide = true
    else
        scene.collide = false
    end
    scene.initSuperObj = true


    return nothing
end



function visualizeWorld!(world::Object3D{F}; scene = Scene())::Nothing where F <: Modia3D.VarFloatType
    scene.analysis = Modia3D.KinematicAnalysis
    initAnalysis!(world, scene)
    updatePosition!(world)
    visualize!(Modia3D.renderer[1], 0.0)
    visualize!(Modia3D.renderer[1], 1.0)
    closeAnalysis!(scene)
    return nothing
end


function makeTreeAvailable(scene::Scene{F})::Nothing where F <: Modia3D.VarFloatType
    if scene.options.useOptimizedStructure
        scene.treeForComputation = scene.treeAccVelo
    else
        for obj in scene.tree
            if featureHasMass(obj.feature)
                # Copy feature mass properties to object mass properties
                obj.m    = obj.feature.massProperties.m
                obj.r_CM = obj.feature.massProperties.rCM
                obj.I_CM = obj.feature.massProperties.I
            end
        end
        scene.treeForComputation = scene.tree
    end
    return nothing
end


function chooseAndBuildUpTree(world::Object3D{F}, scene::Scene{F}) where F <: Modia3D.VarFloatType
    # Build tree for optimized structure or standard structure
    # collision handling is only available for optimized structure
    if scene.options.useOptimizedStructure
        build_superObjs!(scene, world)
        if scene.options.enableContactDetection && scene.collide
            initializeContactDetection!(world, scene)
            if length(world.contactVisuObj1) > 0
                append!(scene.visualObject3Ds, world.contactVisuObj1)
                append!(scene.visualObject3Ds, world.contactVisuObj2)
            end
            if length(world.supportVisuObj1A) > 0
                append!(scene.visualObject3Ds, world.supportVisuObj1A)
                append!(scene.visualObject3Ds, world.supportVisuObj2A)
                append!(scene.visualObject3Ds, world.supportVisuObj3A)
                append!(scene.visualObject3Ds, world.supportVisuObj1B)
                append!(scene.visualObject3Ds, world.supportVisuObj2B)
                append!(scene.visualObject3Ds, world.supportVisuObj3B)
            end
            if length(world.AABBVisu) > 0
                append!(scene.visualObject3Ds, world.AABBVisu)
            end
        end
        initializeMassComputation!(scene)
    else
        build_tree!(scene, world)
        if scene.options.enableContactDetection
            error("Collision handling is only possible with the optimized structure. Please set useOptimizedStructure = true in Modia3D.Composition.Scene.")
        end
    end
    # set visualize
    if F <: MonteCarloMeasurements.StaticParticles ||
    F <: MonteCarloMeasurements.Particles
        @warn("For MonteCarloMeasurements visualization and animation export is not supported.")
        scene.visualize = false
        scene.exportAnimation = false
    else
        if length(scene.visualObject3Ds) > 0
            scene.visualize = scene.options.enableVisualization
        else
            scene.visualize = false
            scene.exportAnimation = false
        end
    end
    makeTreeAvailable(scene)
    scene.initAnalysis = true
    return nothing
end


function initAnalysis!(world::Object3D{F}, scene::Scene{F}) where F <: Modia3D.VarFloatType
    # Initialize spanning tree and visualization (if visualization desired and visual elements present)
    chooseAndBuildUpTree(world, scene)
    if scene.visualize
        initializeVisualization(Modia3D.renderer[1], scene)
    end
    return nothing
end


function errorMessageCollision(functionName::String)
    error("\nError message from ", functionName, ":\n",
            "There are too less solids which can collide.")
end


function visualize!(scene::Scene{F}, time)::Nothing where F <: Modia3D.VarFloatType
    if scene.visualize
        visualize!(Modia3D.renderer[1], time)
    end
    return nothing
end


function emptyScene!(scene::Scene{F})::Nothing where F <: Modia3D.VarFloatType
    empty!(scene.stack)
    empty!(scene.buffer)
    empty!(scene.superObjs)
    empty!(scene.userDefinedObject3Ds)
    empty!(scene.visualObject3Ds)
    empty!(scene.pureResultObject3Ds)
    empty!(scene.allCollisionElements)
    empty!(scene.treeAccVelo)
    empty!(scene.tree)
    empty!(scene.noCPairs)
    empty!(scene.noCPairsHelp)
    empty!(scene.allowedToMove)
    empty!(scene.AABB)

    scene.collide = false
    scene.initSuperObj = false

    # Close Analysis
    scene.initAnalysis = false
    return nothing
end


function closeAnalysis!(scene::Scene{F})::Nothing where F <: Modia3D.VarFloatType
    # Close Visualisation
    closeVisualization(Modia3D.renderer[1])
    empty!(scene.visualObject3Ds)
    scene.visualize = false
    # Close Collision detection
    if scene.collide
        closeContactDetection!(scene.options.contactDetection)
    end
    emptyScene!(scene)
    return nothing
end
