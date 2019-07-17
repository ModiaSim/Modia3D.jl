module Simulate_PathPlanning

using  Modia3D


const ptp_path = PTP_path(["angle1", "angle2", "angle3"],
                          positions = [0.0 2.0 3.0;
                                       0.5 3.0 4.0;
                                       0.8 1.5 0.3;
                                       0.2 1.5 0.8],
                          startTime = 0.1,
                          v_max = 2*ones(3),
                          a_max = 3*ones(3))

plotPath(ptp_path)

plotPath(ptp_path, names=("angle2", "angle3"), heading="Test of PTP plots",
         tend = 0.9*ptp_path.Tend, figure=2, ntime=99)

end