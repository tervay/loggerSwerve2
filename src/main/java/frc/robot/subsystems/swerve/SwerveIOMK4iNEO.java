package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

public class SwerveIOMK4iNEO implements SwerveIO {

    Pigeon2 gyro;

    public SwerveIOMK4iNEO() {
        gyro = new Pigeon2(0);
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.gyroHeadingRad = gyro.getYaw();
    }

    @Override
    public void resetGyro() {
        gyro.setYaw(0);
    }
}
