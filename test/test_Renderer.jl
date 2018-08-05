module TestRenderer

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end

import Modia3D
using StaticArrays

sphere = Modia3D.Object3D( Modia3D.Sphere(0.1) )
velements = Modia3D.Object3D[]
push!(velements, sphere)

renderer = Modia3D.DLR_Visualization.simVisInfo.isNoRenderer ? Modia3D.Composition.NoRenderer() : Modia3D.Composition.DLR_Visualization_renderer()

Modia3D.Composition.initializeVisualization(renderer,velements)

Modia3D.Composition.visualize!(renderer, 0.0)

#Profile.clear_malloc_data()
@time Modia3D.Composition.visualize!(renderer, 0.0)

Modia3D.Composition.closeVisualization(renderer)

end
