# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Shapes (Modia3D/Solids/_module.jl)
#

# This algorithm accords with [D. Eberly (2002), Polyhedral Mass Properties (Revisited)]
# https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
# it returns volume, centroid and inertia of a polyhedral shape.
# it input values are:
#   vertices          ... all nodes of an object
#   triangle_indices  ... indices of faces, which span the surface.
#                         All faces should be specified in right-handed/counter-clockwise order!


neps = sqrt(eps())

function helpingFunc(w0,w1,w2)
    tmp0 = w0 + w1
    f1   = tmp0 + w2
    tmp1 = w0.*w0
    tmp2 = tmp1 + w1.*tmp0
    f2   = tmp2 + w2.*f1
    f3   = w0.*tmp1 + w1.*tmp2 + w2.*f2
    g0   = f2 + w0.*(f1+w0)
    g1   = f2 + w1.*(f1+w1)
    g2   = f2 + w2.*(f1+w2)
    return (f1,f2,f3,g0,g1,g2)
end


function computeMassProperties(vertices, triangle_indices; bodyCoords::Bool=false)
    if isempty(triangle_indices)
        # (volume, centroid, inertia)
        return (0.0, zeros(3), zeros(9))
    else
    weights = [1/6, 1/24, 1/24, 1/24, 1/60, 1/60, 1/60, 1/120, 1/120, 1/120]
    integral = zeros(10)
    volume     = 0.0
    centroid       = zeros(3)
    inertia  = SMatrix{3,3,Float64,9}(zeros(9))
    for i=eachindex(triangle_indices)
        # get vertices of i-th triangle
        v1 = vertices[triangle_indices[i][1]]
        v2 = vertices[triangle_indices[i][2]]
        v3 = vertices[triangle_indices[i][3]]

        # get edges and cross products of edges
        v1v2 = v2 - v1
        v1v3 = v3 - v1
        n    = cross(v1v2, v1v3) # )

        (f1,f2,f3,g0,g1,g2) = helpingFunc(v1, v2, v3)

        integral[1] += n[1] * f1[1]
        integral[2] += n[1] * f2[1]
        integral[3] += n[2] * f2[2]
        integral[4] += n[3] * f2[3]
        integral[5] += n[1] * f3[1]
        integral[6] += n[2] * f3[2]
        integral[7] += n[3] * f3[3]
        integral[8] += n[1] * (v1[2]*g0[1] + v2[2]*g1[1] + v3[2]*g2[1])
        integral[9] += n[2] * (v1[3]*g0[2] + v2[3]*g1[2] + v3[3]*g2[2])
        integral[10] += n[3] * (v1[1]*g0[3] + v2[1]*g1[3] + v3[1]*g2[3])
    end

    for i=1:10
        integral[i] *= weights[i]
    end

    volume = integral[1]
    if volume > neps
        # center of volume
        centroid = [integral[2], integral[3], integral[4]]/volume

        # inertia tensor
        inertia_xx = integral[6] + integral[7]
        inertia_yy = integral[5] + integral[7]
        inertia_zz = integral[5] + integral[6]
        inertia_xy = -integral[8]
        inertia_yz = -integral[9]
        inertia_xz = -integral[10]

        if bodyCoords
            inertia_xx -= volume*(centroid[2]^2 + centroid[3]^2)
            inertia_yy -= volume*(centroid[3]^2 + centroid[1]^2)
            inertia_zz -= volume*(centroid[1]^2 + centroid[2]^2)
            inertia_xy += volume*centroid[1]*centroid[2]
            inertia_yz += volume*centroid[2]*centroid[3]
            inertia_xz += volume*centroid[3]*centroid[1]
        end
        inertia = SMatrix{3,3,Float64,9}([inertia_xx inertia_xy inertia_xz; inertia_xy inertia_yy inertia_yz; inertia_xz inertia_yz inertia_zz])
    end
    return (volume, centroid, inertia)
end; end
