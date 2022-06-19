package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveIOPigeon2 implements SwerveIO {

    Pigeon2 gyro;

    public SwerveIOPigeon2() {
        gyro = new Pigeon2(0);
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.gyroHeadingRad = gyro.getYaw();
    }

    @Override
    public void resetGyro(Rotation2d rot) {
        gyro.setYaw(rot.getDegrees());
    }
}
