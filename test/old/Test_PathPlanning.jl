module Test_PathPlanning

import Modia3D
using  ModiaLang
@usingModiaPlot

const ptp_path = Modia3D.PTP_path(["angle1", "angle2", "angle3"],
                                  positions=[0.0 2.0 3.0;
                                             0.5 3.0 4.0;
                                             0.8 1.5 0.3;
                                             0.2 1.5 0.8],
                                  startTime=0.1,
                                  v_max=2*ones(3),
                                  a_max=3*ones(3))

Modia3D.plotPath(ptp_path, plot)

Modia3D.plotPath(ptp_path, plot, names=("angle2", "angle3"), heading="Test of PTP plots",
                 tend=0.9*ptp_path.Tend, figure=2, ntime=99)

Modia3D.plotPath(ptp_path, plot, onlyPositions=false, ntime=1000, figure=3)

Modia3D.plotPath(ptp_path, plot, names=("angle2", "angle3"), heading="Test of PTP plots",
                 tend = 0.9*ptp_path.Tend, onlyPositions=false, ntime=1000, figure=4)

# Test getPosition
angles = zeros(3)
Modia3D.getPosition!(ptp_path, 0.5, angles)
println("... angles(time=0.5) = $angles")

angle2 = Modia3D.getPosition(ptp_path, 2, 0.5)
println("... angle2(time=0.5) = ", angle2)

der_angles  = zeros(3)
der2_angles = zeros(3)
Modia3D.getPosition!(ptp_path, 0.5, angles, der_angles, der2_angles)
println("... angles(          time=0.5) = $angles")
println("... der(angles)(     time=0.5) = $der_angles")
println("... der(der(angles))(time=0.5) = $der2_angles")

end
