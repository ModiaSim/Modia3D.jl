function printObj3DTree(scene::Scene; world::Union{Nothing, Object3D}=nothing)
    printObjInfos(world)
    printTree(scene.treeAccVelo)
end


function printAllVisuElements(scene::Scene)
    printTree(scene.allVisuElements)
end


function printTree(tree)
    for obj in tree
        printObjInfos(obj)
    end
end


printObjInfos(obj::Nothing) = nothing
function printObjInfos(obj::Object3D)
    println("parent   = ", Modia3D.fullName(obj.parent))
    println("Object3D = ", Modia3D.fullName(obj))
    println("children:")
    for child in obj.children
        println("  child = ", Modia3D.fullName(child))
    end
    println("joint   = ", obj.joint)
    println(" ")
end



function printScene(scene)
    println("\nScene")
    tree = scene.options.useOptimizedStructure ? scene.treeAccVelo : scene.tree
    if hasParent(tree[1])
        println("  ", tree[1].parent.path)
    end
    for obj in tree
        println("  ", obj.path)
        if hasParent(obj)
            println("    parent = ", obj.parent.path)
        end
        println("    r_rel = ", obj.r_rel)
        print("    joint")
        if obj.joint.path != ""
            println(" (= ", obj.joint.path, ")")
        else
            println()
        end
        println("      kind       = ", obj.joint.kind)
        println("      ndof       = ", obj.joint.ndof)
        println("      canCollide = ", obj.joint.canCollide)
        if obj.joint.kind == RevoluteKind
            println("      phi        = ", obj.joint.specific.phi)
            println("      w          = ", obj.joint.specific.w)
        elseif obj.joint.kind == PrismaticKind
            println("      s          = ", obj.joint.specific.s)
            println("      v          = ", obj.joint.specific.v)
        elseif obj.joint.kind == FreeMotionKind || obj.joint.kind == AbsoluteFreeMotionKind
            println("      r          = ", obj.joint.specific.r)
            println("      rot        = ", obj.joint.specific.rot)
            println("      v          = ", obj.joint.specific.v)
            println("      w          = ", obj.joint.specific.w)
        end
    end
    println()
end
