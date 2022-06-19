package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

public class SwerveIOSim implements SwerveIO {

    AnalogGyro gyro = new AnalogGyro(0);
    AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.gyroHeadingRad = gyro.getRotation2d().getRadians();
    }

    @Override
    public void resetGyro() {
        gyro.reset();
    }
}
