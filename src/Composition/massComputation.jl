function initializeMassComputation!(scene::Scene)
  superObjs = scene.superObjs
  buffer    = scene.buffer

  for i = 1:length(superObjs)
    rootSuperObj = buffer[i]
    if dataHasMass(rootSuperObj)
      println("dataHasMass(rootSuperObj) = ", dataHasMass(rootSuperObj))
    else
      println("vorher objectHasMass(rootSuperObj) = ", objectHasMass(rootSuperObj))
      rootSuperObj.massProperties = Solids.dummyMassProperties()
      println("nachher objectHasMass(rootSuperObj) = ", objectHasMass(rootSuperObj))
      for obj in superObjs[i].superObjMass.superObj
        println(ModiaMath.fullName(a))
      end
      # println("objectHasMass(rootSuperObj) = ", objectHasMass(rootSuperObj))
    end
    println(" ")
  end


  #=
    for superObjRow in scene.superObjs
      println("[")
      for a in superObjRow.superObjMass.superObj
        println(ModiaMath.fullName(a))
      end
      println("]")
      println(" ")
    end
=#
end
