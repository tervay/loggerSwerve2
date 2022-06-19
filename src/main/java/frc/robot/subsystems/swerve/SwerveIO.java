package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveIO {

    @AutoLog
    public static class SwerveIOInputs {
        public double gyroHeadingRad = 0.0;
    }

    public void updateInputs(SwerveIOInputs inputs);

    public void resetGyro();
}
