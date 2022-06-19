package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.module.ModuleIO.ModuleIOInputs;

public class Module extends SubsystemBase {

    private final ModuleIO io;
    private final ModuleIOInputs inputs = new ModuleIOInputs();

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(1.0, 1.0));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    public Module(ModuleIO io) {
        this.io = io;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.wheelVelocityMetersPerSec,
                new Rotation2d(inputs.azimuthEncoderPositionRads));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(inputs.azimuthEncoderPositionRads));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(inputs.wheelVelocityMetersPerSec,
                state.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(inputs.azimuthEncoderPositionRads,
                state.angle.getRadians());

        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        io.setWheelVolts(driveOutput + driveFeedforward);
        io.setAzimuthVolts(turnOutput + turnFeedforward);
    }

    public ModuleConfig getConfig() {
        return io.getConfig();
    }

    public void periodic() {
        io.updateInputs(inputs);
    }
}
