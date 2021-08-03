module Runtests

using  ModiaLang
import Modia3D
import Test

@time Test.@testset verbose=true "Modia3D (with SilentNoPlot)" begin
    usePlotPackage("SilentNoPlot")
    Modia3D.disableRenderer()
    include("includeTests.jl")
    Modia3D.reenableRenderer()
    usePreviousPlotPackage()
end

end
