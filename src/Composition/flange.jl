function setAnalysis!(var::ModiaMath.RealScalar)
  if var.causality == ModiaMath.Local
    var.analysis = ModiaMath.NotUsedInAnalysis
  end
end

function setCausality!(varJoint::ModiaMath.RealScalar, varFlange::ModiaMath.RealScalar)
  varJoint.causality = varFlange.causality
end

function setFlow!(varJoint::ModiaMath.RealScalar, varFlange::ModiaMath.RealScalar)
  (varFlange.flow ? varJoint.flow=true : varJoint.flow=false)
end

# assigns and inverts causality for revoluteFlange if it gets connected with a flange
function invertCausality(varFlange::ModiaMath.RealScalar, varFlangeRev::ModiaMath.RealScalar)
  if varFlange.causality == ModiaMath.Input
    varFlangeRev.causality = ModiaMath.Output
  elseif varFlange.causality == ModiaMath.Output
    varFlangeRev.causality = ModiaMath.Input
  end
  (varFlange.flow ? varFlangeRev.flow=true : varFlangeRev.flow=false)
end

function setFlangeVariable!(varFlange::ModiaMath.RealScalar, varFlangeRev::ModiaMath.RealScalar)
  invertCausality(varFlange, varFlangeRev)
  setAnalysis!(varFlange)
end

function setJointVariable!(varJoint::ModiaMath.RealScalar, varFlange::ModiaMath.RealScalar)
  setCausality!(varJoint,varFlange)
  setFlow!(varJoint,varFlange)
end

# checks causalities of flangeA and flangeB, it only returns true, if both are not defined or different
function causalitiesAreOk(variableFlangeA::ModiaMath.RealScalar, variableFlangeB::ModiaMath.RealScalar)
    if variableFlangeA.causality == variableFlangeB.causality == ModiaMath.Local
        return true
    elseif variableFlangeA.causality == variableFlangeB.causality
        return false
    else
        return true
    end
end

#------------------------------

# each revolute joint holds a RevoluteFlange
# it is an internal flange for connecting with a Flange
mutable struct RevoluteFlange <: Modia3D.AbstractFlange
  phi::ModiaMath.RealScalar
  w::ModiaMath.RealScalar
  a::ModiaMath.RealScalar
  tau::ModiaMath.RealScalar
  function RevoluteFlange()
    phi = ModiaMath.RealScalar("phi_flange", causality = ModiaMath.Local, flow=false)
    w   = ModiaMath.RealScalar("w_flange",   causality = ModiaMath.Local, flow=false)
    a   = ModiaMath.RealScalar("a_flange",   causality = ModiaMath.Local, flow=false)
    tau = ModiaMath.RealScalar("tau_flange", causality = ModiaMath.Local, flow=false)
    setAnalysis!(phi)
    setAnalysis!(w)
    setAnalysis!(a)
    setAnalysis!(tau)
    new(phi,w,a,tau)
  end
end

# each adaptor holds a Flange
# if is an internal flange for connecting with e.g. a RevoluteFlange
mutable struct Flange <: Modia3D.AbstractFlange
  phi::ModiaMath.RealScalar
  w::ModiaMath.RealScalar
  a::ModiaMath.RealScalar
  tau::ModiaMath.RealScalar
  function Flange(; phiCausality=ModiaMath.Local, wCausality=ModiaMath.Local,
                    aCausality=ModiaMath.Local, phiFlow=false, tauCausality=ModiaMath.Local, tauFlow=false)
    phi = ModiaMath.RealScalar("phi_flange", causality = phiCausality, flow=phiFlow)
    w   = ModiaMath.RealScalar("w_flange",   causality = wCausality,   flow=phiFlow)
    a   = ModiaMath.RealScalar("a_flange",   causality = aCausality,   flow=phiFlow)
    tau = ModiaMath.RealScalar("tau_flange", causality = tauCausality, flow=tauFlow)
    setAnalysis!(phi)
    setAnalysis!(w)
    setAnalysis!(a)
    setAnalysis!(tau)
    new(phi,w,a,tau)
  end
end



# this function checks causality of a revolute flange
function checkCausalityOfOneFlange(flange::RevoluteFlange)
  potentialCausality = ModiaMath.Local
  flowCausality = flange.tau.causality
  names = fieldnames(typeof(flange))

  for val in names
      if val != :tau && val != :isInitialized
        tmp = getfield(flange, val)
        if !(potentialCausality == tmp.causality || tmp.causality == ModiaMath.Local || potentialCausality != ModiaMath.Local)
            potentialCausality = tmp.causality
        elseif (potentialCausality != ModiaMath.Local && potentialCausality != tmp.causality && tmp.causality != ModiaMath.Local)
            println("All potential variables of a flange must be of the same causality.")
        end
      end
  end
  if potentialCausality == flowCausality && flowCausality != ModiaMath.Local
      error("Causality of flow and potential variables must be different.")
  end
  return (potentialCausality, flowCausality)
end

function isPhiInput(flange::RevoluteFlange)
  (potentialCausality, flowCausality) = checkCausalityOfOneFlange(flange)
  ((potentialCausality == ModiaMath.Input) ? (return true) : (return false) )
end

function isTauInput(flange::RevoluteFlange)
  (potentialCausality, flowCausality) = checkCausalityOfOneFlange(flange)
  ((flowCausality == ModiaMath.Input) ? (return true) : (return false) )
end
