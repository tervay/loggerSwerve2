package frc.robot.subsystems.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveIO.SwerveIOInputs;
import frc.robot.subsystems.swerve.SwerveMovementMode.MovementMode;
import frc.robot.subsystems.swerve.module.ModuleContainer;

public class Swerve extends SubsystemBase {

    private final SwerveIO io;
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();
    ModuleContainer modules;
    private final double kMaxSpeed = Units.feetToMeters(16);

    private final SwerveDriveOdometry odometry;
    private Pose2d pose = new Pose2d();

    private static final double xTuningGoal = 11;
    private static final double yTuningGoal = 11;
    private static final double rTuningGoal = Rotation2d.fromDegrees(90).getRadians();

    private MovementMode movementMode;

    public Swerve(SwerveIO io, ModuleContainer modules) {
        this.io = io;
        this.modules = modules;

        io.resetGyro(new Rotation2d());
        io.updateInputs(inputs);
        odometry = new SwerveDriveOdometry(Constants.kKinematics, new Rotation2d(inputs.gyroHeadingRad));
        movementMode = MovementMode.DRIVER;
    }

    public void setStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        modules.getFrontLeft().setDesiredState(swerveModuleStates[0]);
        modules.getFrontRight().setDesiredState(swerveModuleStates[1]);
        modules.getBackLeft().setDesiredState(swerveModuleStates[2]);
        modules.getBackRight().setDesiredState(swerveModuleStates[3]);
    }

    public void setMovementMode(MovementMode mode) {
        this.movementMode = mode;
    }

    public void updateOdometry() {
        odometry.update(
                new Rotation2d(inputs.gyroHeadingRad),
                modules.getFrontLeft().getState(),
                modules.getFrontRight().getState(),
                modules.getBackLeft().getState(),
                modules.getBackRight().getState());

        // ----
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {
            measuredStates[i] = new SwerveModuleState(
                    modules.asList().get(i).getInputs().wheelVelocityMetersPerSec,
                    Rotation2d.fromDegrees(modules.asList().get(i).getInputs().azimuthEncoderPositionDeg));
        }

        ChassisSpeeds speeds = Constants.kKinematics.toChassisSpeeds(measuredStates);
        Logger.getInstance().recordOutput("Sim omega rad/sec", speeds.omegaRadiansPerSecond);
        pose = pose.exp(new Twist2d(
                speeds.vxMetersPerSecond * .02,
                speeds.vyMetersPerSecond * .02,
                speeds.omegaRadiansPerSecond * .02));
    }

    public Pose2d getPose() {
        // return odometry.getPoseMeters();
        return pose;
    }

    public void setPose(Pose2d pose) {
        // odometry.resetPosition(pose, pose.getRotation());
        this.pose = pose;
        io.resetGyro(pose.getRotation());
    }

    public ModuleContainer getModules() {
        return modules;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Swerve", inputs);

        updateOdometry();

        Logger.getInstance().recordOutput("Odometry", new double[] {
                getPose().getX(),
                getPose().getY(),
                getPose().getRotation().getRadians(),
        });
        Logger.getInstance().recordOutput("Swerve/X Tuning Error", xTuningGoal - odometry.getPoseMeters().getX());
        Logger.getInstance().recordOutput("Swerve/Y Tuning Error", yTuningGoal - odometry.getPoseMeters().getY());
        Logger.getInstance().recordOutput("Swerve/R Tuning Error", rTuningGoal - getPose().getRotation().getRadians());

        
        setStates(SwerveMovementMode.handle(movementMode));
    }

    public void followTrajectory(PathPlannerTrajectory t) {
        setMovementMode(MovementMode.FOLLOW_TRAJECTORY);
        setPose(t.getInitialHolonomicPose());
        TrajectoryController.getInstance().setTrajectory(t);
    }

    @Override
    public void simulationPeriodic() {
    }
}
