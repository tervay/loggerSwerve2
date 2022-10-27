package frc.robot.subsystems.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.PIDFFController;
import frc.robot.PIDFFGains;
import frc.robot.Robot;

public class TrajectoryController {
    private static TrajectoryController instance;

    private PathPlannerTrajectory trajectory;
    private PPHolonomicDriveController holoController;
    private Timer trajectoryTimer;
    private boolean running;

    private TrajectoryController() {
        holoController = new PPHolonomicDriveController(
                new PIDFFController(PIDFFGains.builder("Holo X").kP(1.5).build()),
                new PIDFFController(PIDFFGains.builder("Holo Y").kP(1.5).build()),
                new PIDFFController(PIDFFGains.builder("Holo R").build()));
        trajectoryTimer = new Timer();
        running = false;
    }

    public static TrajectoryController getInstance() {
        if (instance == null) {
            instance = new TrajectoryController();
        }

        return instance;
    }

    private void startIfNotStarted() {
        if (!running) {
            running = true;
            trajectoryTimer.start();
        }
    }

    public void setTrajectory(PathPlannerTrajectory t) {
        trajectory = t;
        trajectoryTimer.stop();
        trajectoryTimer.reset();
        running = false;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        if (trajectory == null) {
            running = false;
            return Constants.kKinematics.toSwerveModuleStates(new ChassisSpeeds());
        }

        if (trajectoryTimer.hasElapsed(trajectory.getTotalTimeSeconds())) {
            trajectoryTimer.stop();
            return Constants.kKinematics.toSwerveModuleStates(new ChassisSpeeds());
        }

        startIfNotStarted();
        PathPlannerState targetState = (PathPlannerState) trajectory.sample(trajectoryTimer.get());
        Pose2d currentPose = Robot.swerve.getPose();

        ChassisSpeeds targetChassisSpeeds = holoController.calculate(currentPose, targetState);
        SwerveModuleState[] targetModuleStates = Constants.kKinematics.toSwerveModuleStates(targetChassisSpeeds);
        return targetModuleStates;
    }
}
