package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveMovementMode {
    public enum MovementMode {
        DRIVER, MAINTAIN_HEADING, FOLLOW_TRAJECTORY
    }

    public static SwerveModuleState[] handle(MovementMode mode) {
        switch (mode) {
            case DRIVER:
                return handleDriver();
            case MAINTAIN_HEADING:
                return handleMaintainHeading();
            case FOLLOW_TRAJECTORY:
                return handleFollowTrajectory();
            default:
                return Constants.kKinematics.toSwerveModuleStates(new ChassisSpeeds());
        }
    }

    public static SwerveModuleState[] handleDriver() {
        return Constants.kKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        Robot.driver.getLeftX() * Constants.kMaxTranslationalSpeedMetersPerSec,
                        Robot.driver.getLeftY() * Constants.kMaxTranslationalSpeedMetersPerSec,
                        Robot.driver.getRightX()
                                * Rotation2d.fromDegrees(Constants.kMaxTranslationalSpeedMetersPerSec).getRadians(),
                        Robot.swerve.getPose().getRotation()));
    }

    public static SwerveModuleState[] handleMaintainHeading() {
        return null;
    }

    public static SwerveModuleState[] handleFollowTrajectory() {
        return TrajectoryController.getInstance().getSwerveModuleStates();
    }

}
