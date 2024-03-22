function updatedPose = KalmanFilter(pose, vu, omega)
    dt = 0.1;
    A = [1, 0, -vu / omega * cos(pose(3)) + vu / omega * cos(pose(3) + omega * dt);
         0, 1, -vu / omega * sin(pose(3)) + vu / omega * sin(pose(3) + omega * dt);
         0, 0, 1];
    B = [(-sin(pose(3)) + sin(pose(3) + omega * dt)) / omega, vu / (omega^2) * (sin(pose(3)) - sin(pose(3) + omega * dt)) + vu / omega * cos(pose(3) + omega * dt);
         (cos(pose(3)) - cos(pose(3) + omega * dt)) / omega, -vu / (omega^2) * (cos(pose(3)) - cos(pose(3) + omega * dt)) + vu / omega * sin(pose(3) + omega * dt);
         0, dt];
    u = [vu; omega];
    updatedPose = A * pose + B * u;
end