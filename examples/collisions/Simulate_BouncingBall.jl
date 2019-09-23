"""
    module Simulate_BouncingBall

Simulation of a Modia3D model of a ball, bouncing on ground using an
elastic response calculation. The result is compared
with the simulation of a direct implementation of this system in ModiaMath
using an impulsive response calculation.

There is a bug with the event handling (if interval=0.01 is selected, the simulation is wrong.)
"""
module Simulate_BouncingBall

using  Modia3D
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

    function Model(;h0=1.0, cor=0.7, g=9.81, vsmall=0.1)
        @assert(h0 > 0.0)
        @assert(0.0 <= cor <= 1.0)
        @assert(vsmall > 0.0)
        simulationState = ModiaMath.SimulationState("BouncingBall1", getModelResidues!, [h0;0.0], getVariableName;
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
        println("... h0 = ", _x[1])
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

    # println("+++ time = ", t, ", h = ", h)
    if ModiaMath.edge!(sim, 1, -h, "-h")
        cor_res = Modia3D.resultantCoefficientOfRestitution(m.cor, abs(v), m.vsmall, cor_min=0.0)
        _w[1] =  cor_res
            v = -cor_res * v
        _x[2] =  v    # re-initialize state vector x
        println("... v = $v")
    end
#=
    if ModiaMath.edge!(sim, 2, v, "v")
        if h <= 0.0
            m.flying = false
            v = 0.0
            if ModiaMath.isLogInfos(sim)
                println("        flying = ", m.flying)
            end
            _x[2] = v    # re-initialize state vector x
        end
    end
=#

    derh = v
    derv = m.flying ? -m.g : 0.0

    _r[1] = derh - _derx[1]
    _r[2] = derv - _derx[2]
    _w[2] = m.flying

    return nothing
end

cor      = 0.7
vsmall   = 0.1
radius   = 0.01
h0       = 20*radius
stopTime = 1.0

model1  = Model(h0=h0, cor=cor, vsmall=vsmall )
#result1 = ModiaMath.simulate!(model1, stopTime=stopTime, log=true)
result1 = ModiaMath.simulate!(model1, stopTime=stopTime, interval=0.02, log=true)  # interval=0.01 gives wrong behaviour
ModiaMath.plot(result1, ["h", ("v", "flying"), "cor_res"], heading="Bouncing ball with impulsive response calculation", figure=1)

ballMaterial  = Modia3D.Material(color="Red"       , transparency=0.5)
tableMaterial = Modia3D.Material(color="LightBlue" , transparency=0.5)

@assembly BouncingBall2 begin
  world = Object3D()
  ball  = Object3D(world, Solid(SolidSphere(2*radius) , "Steel"  , ballMaterial ; contactMaterial="Steel"); r=[0.0, 0.0, h0], fixed=false)
  box   = Object3D(world, Solid(SolidBox(20*radius, 20*radius, 2*radius), "DryWood", tableMaterial; contactMaterial="Steel");
                   r=[0.0, 0.0,-2*radius], fixed=true)
end

gravField     = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1.0])
bouncingBall2 = BouncingBall2(sceneOptions=Modia3D.SceneOptions(gravityField=gravField, visualizeFrames=false,
                                                                defaultFrameLength=0.1, enableContactDetection=true))
model2        = SimulationModel( bouncingBall2 )
result2       = ModiaMath.simulate!(model2; stopTime=stopTime, tolerance=1e-8, log=false)
ModiaMath.plot(result2, ["ball.r[3]", "ball.v[3]"], heading="Bouncing ball with compliant response calculation", figure=2)


# Need to be included in ModiaMath
import Base
function Base.getproperty(result::ModiaMath.ResultWithVariables, name::AbstractString)
    (sig, dummy1, dummy2) = ModiaMath.Result.getSignal(result.series, name)
    return sig
end

using PyPlot
#=
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
=#


t1      = result1["time"]
h1      = result1["h"]
v1      = result1["v"]
cor_res = result1["cor_res"]

t2 = result2."time"
h2 = result2."ball.r[3]"
v2 = result2."ball.v[3]"

figure(3)
clf()

#=
plot(t1,h1,"b", t2,h2,"r", t1,cor_res,"g")
grid(true)
xlabel("time [s]")
ylabel("height [m]")
legend(["impulsive response",
        "compliant response",
        "\$cor_{res}\$"])
=#
plot(t1,h1,"b", t2,h2,"r")
grid(true)
xlabel("time [s]")
ylabel("height [m]")
legend(["impulsive response",
        "compliant response"])

println("... success of examples/collisions/Simulate_BouncingBall.jl")
end
