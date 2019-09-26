# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control

"""
    module BouncingBallComparedWithImpulses

Model of a ball bouncing on ground simulated with Modia3D. The result is compared
with the simulation of a direct implementation in ModiaMath using an impulsive response calculation.
"""
module BouncingBallComparedWithImpulses

using Modia3D
import Modia3D.ModiaMath

mutable struct Model <: ModiaMath.AbstractSimulationModel
    simulationState::ModiaMath.SimulationState

    # Parameters
    h0::Float64   # initial height
    cor::Float64    # coefficient of restitution
    g::Float64    # gravity constant
    vsmall::Float64

    # Discrete variables
    flying::Bool

    function Model(;h0=1.0, cor=0.7, g=9.81, vsmall=0.01)
        @assert(h0 > 0.0)
        @assert(0.0 <= cor <= 1.0)
        @assert(vsmall > 0.0)
        simulationState = ModiaMath.SimulationState("BouncingBall", getModelResidues!, [h0;0.0], getVariableName;
                                                    nz=1, nw=2)
        new(simulationState, h0, cor, g, vsmall, true)
    end
end

getVariableName(model, vcat, vindex) = ModiaMath.getVariableName(model, vcat, vindex;
                                                                 xNames=["h", "v"],
                                                                 wNames=["cor_res", "flying"])

function getModelResidues!(m::Model, t::Float64, _x::Vector{Float64}, _derx::Vector{Float64}, _r::Vector{Float64}, _w::Vector{Float64})
    sim = m.simulationState
    if ModiaMath.isInitial(sim)
        if ModiaMath.isLogInfos(sim)
            println("        flying = ", m.flying)
        end
        _w[1] = m.cor
    end
    if ModiaMath.isTerminal(sim)
        if ModiaMath.isLogInfos(sim)
            println("\n      BouncingBall model is terminated (flying = ", m.flying, ")")
        end
        return
    end

    h = _x[1]
    v = _x[2]

    if ModiaMath.edge!(sim, 1, -h, "-h")
        cor_res = Modia3D.resultantCoefficientOfRestitution(m.cor, abs(v), m.vsmall, cor_min=0.0)
        _w[1] = cor_res
        v = -cor_res * v
        if v < m.vsmall
        #if cor_res <= 0.01
            m.flying = false
            v = 0.0
            if ModiaMath.isLogInfos(sim)
                println("        flying = ", m.flying)
            end
        end
        _x[2] = v    # re-initialize state vector x
    end

    derh = v
    derv = m.flying ? -m.g : 0.0

    _r[1] = derh - _derx[1]
    _r[2] = derv - _derx[2]
    _w[2] = m.flying

    return nothing
end

material = 
cmat = "Steel"


cor = 0.7
vsmall = 0.01
#cpairs = Modia3D.getCommonCollisionProperties(material,material)
#cpairs.cor = cor

#cpairs2 = Modia3D.getCommonCollisionProperties(material,material)
#println("cpairs2 = ", cpairs2)

stopTime = 3.0


model1 = Model(cor=cor, vsmall=vsmall )
result1 = ModiaMath.simulate!(model1, stopTime=stopTime, log=false)
ModiaMath.plot(result1, ["h", ("v", "flying"), "cor_res"], heading="Bouncing ball with impulsive response calculation", figure=1)

ballMaterial  = Modia3D.Material(color="Red"       , transparency=0.5)
tableMaterial = Modia3D.Material(color="LightBlue" , transparency=0.5)

@assembly BouncingBall2 begin
  world = Modia3D.Object3D(visualizeFrame=false)
  ball  = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(0.1)      , "Steel", ballMaterial ; contactMaterial = cmat); r=[0.0, 0.0, 1.0], fixed=false)
  box   = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(1.0,1.0,0.1) , "Steel", tableMaterial; contactMaterial = cmat); r=[0.0, 0.0,-0.1], fixed=true)
end

#=
@assembly BouncingBall2 begin
  world = Modia3D.Object3D(visualizeFrame=true)
  ball  = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(0.1)      , "Steel", vmat); r=[0.0, 0.0, 1.0], fixed=false)
  box   = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidBox(1.0,1.0,0.1) , "Steel", vmat); r=[0.0, 0.0,-0.1], fixed=true)
end
=#

gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1.0])
bouncingBall2 = BouncingBall2(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true))
model2        = Modia3D.SimulationModel( bouncingBall2 )
result2       = ModiaMath.simulate!(model2; stopTime=stopTime, tolerance=1e-8,interval=0.001, log=true)
ModiaMath.plot(result2, ["ball.r[3]", "ball.v[3]"], heading="Bouncing ball with compliant response calculation", figure=2)


# Need to be included in ModiaMath
import Base
function Base.getproperty(result::ModiaMath.ResultWithVariables, name::AbstractString)
    (sig, dummy1, dummy2) = ModiaMath.Result.getSignal(result.series, name)
    return sig
end


using PyPlot
using PyCall


fig, ax = PyPlot.subplots(figsize=(3,9))

pyplot_rc = PyCall.PyDict(PyPlot.matplotlib."rcParams")
pyplot_rc["font.family"]      = "sans-serif"
pyplot_rc["font.sans-serif"]  = ["Calibri", "Arial", "Verdana", "Lucida Grande"]
pyplot_rc["font.size"]        = 12.0
pyplot_rc["lines.linewidth"]  = 1.5
pyplot_rc["grid.linewidth"]   = 0.5
pyplot_rc["axes.grid"]        = true
pyplot_rc["axes.titlesize"]   = "medium"
pyplot_rc["figure.titlesize"] = "medium"



t1      = result1["time"]
h1      = result1["h"]
v1      = result1["v"]
cor_res = result1["cor_res"]

t2 = result2."time"
h2 = result2."ball.r[3]"
v2 = result2."ball.v[3]"

figure(3)
clf()

plot(t1,h1,"b", t2,h2,"r", t1,cor_res,"g")
grid(true)
xlabel("time [s]")
ylabel("height [m]")
legend(["impulsive response",
        "compliant response",
        "\$cor_{res}\$"])

println("... success of bouncingBallComparedWithImpulsesjl!")


end
