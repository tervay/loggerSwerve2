package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveIO {
    public static class SwerveIOInputs implements LoggableInputs {
        public double gyroHeadingRad = 0.0;

        @Override
        public void toLog(LogTable table) {
        }

        @Override
        public void fromLog(LogTable table) {
        }
    }

    public void updateInputs(SwerveIOInputs inputs);
    public void resetGyro();
}
