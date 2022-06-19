package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveIO.SwerveIOInputs;
import frc.robot.subsystems.swerve.module.ModuleContainer;

public class Swerve extends SubsystemBase {
    private final SwerveIO io;
    private final SwerveIOInputs inputs = new SwerveIOInputs();
    ModuleContainer modules;
    private final double kMaxSpeed = Units.feetToMeters(16);

    private final SwerveDriveKinematics kinematics;

    private final SwerveDriveOdometry odometry;

    public Swerve(SwerveIO io, ModuleContainer modules) {
        this.io = io;
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(
                modules.getFrontLeft().getConfig().getLocation(), modules.getFrontRight().getConfig().getLocation(),
                modules.getBackLeft().getConfig().getLocation(), modules.getBackRight().getConfig().getLocation());

        io.resetGyro();
        io.updateInputs(inputs);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(inputs.gyroHeadingRad));
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        Logger.getInstance().recordOutput("Drive/xSpeed", xSpeed);
        Logger.getInstance().recordOutput("Drive/ySpeed", ySpeed);
        Logger.getInstance().recordOutput("Drive/rSpeed", rot);

        var swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        modules.getFrontLeft().setDesiredState(swerveModuleStates[0]);
        modules.getFrontRight().setDesiredState(swerveModuleStates[1]);
        modules.getBackLeft().setDesiredState(swerveModuleStates[2]);
        modules.getBackRight().setDesiredState(swerveModuleStates[3]);
    }

    public void setStates(SwerveModuleState[] swerveModuleStates) {
        modules.getFrontLeft().setDesiredState(swerveModuleStates[0]);
        modules.getFrontRight().setDesiredState(swerveModuleStates[1]);
        modules.getBackLeft().setDesiredState(swerveModuleStates[2]);
        modules.getBackRight().setDesiredState(swerveModuleStates[3]);
    }

    public void updateOdometry() {
        odometry.update(
                new Rotation2d(inputs.gyroHeadingRad),
                modules.getFrontLeft().getState(),
                modules.getFrontRight().getState(),
                modules.getBackLeft().getState(),
                modules.getBackRight().getState());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(pose, pose.getRotation());
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void periodic() {
        io.updateInputs(inputs);
        updateOdometry();
        Logger.getInstance().recordOutput("Odometry", new double[] {
                getPose().getX(),
                getPose().getY(),
                getPose().getRotation().getRadians(),
        });
    }
}
