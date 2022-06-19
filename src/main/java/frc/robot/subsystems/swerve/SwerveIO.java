package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveIO {

    @AutoLog
    public static class SwerveIOInputs {
        public double gyroHeadingRad = 0.0;
    }

    public void updateInputs(SwerveIOInputs inputs);

    public void resetGyro(Rotation2d rot);

    public default void updateSimGyro(Pose2d currentPose, Pose2d previousPose) {
    }
}
