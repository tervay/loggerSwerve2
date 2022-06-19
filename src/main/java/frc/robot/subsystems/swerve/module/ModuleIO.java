package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    public static class ModuleIOInputs implements LoggableInputs {
        public double wheelPositionMeters = 0;
        public double wheelVelocityMetersPerSec = 0;
        public double azimuthEncoderPositionRads = 0;

        public void toLog(LogTable table) {
        }

        public void fromLog(LogTable table) {
        }
    }

    public void updateInputs(ModuleIOInputs inputs);

    public void setWheelVolts(double volts);

    public void setAzimuthVolts(double volts);

    public ModuleConfig getConfig();
}
