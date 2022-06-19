package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        public double wheelPositionMeters = 0;
        public double wheelVelocityMetersPerSec = 0;
        public double wheelMotorVelocityRPM = 0;
        public double wheelVolts = 0;
        public double wheelCurrentAmps = 0;
        public double azimuthEncoderPositionDeg = 0;
        public double azimuthEncoderVelocityDegPerSec = 0;
        public double azimuthMotorVelocityRPM = 0;
        public double azimuthVolts = 0;
        public double azimuthCurrentAmps = 0;
    }

    public void updateInputs(ModuleIOInputs inputs);

    public void setWheelVolts(double volts);

    public void setAzimuthVolts(double volts);

    public ModuleConfig getConfig();
}
