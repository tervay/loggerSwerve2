package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Module extends SubsystemBase {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    SwerveModuleState desiredState = new SwerveModuleState();

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(5, 0, 0);

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
            1,
            0,
            0,
            new TrapezoidProfile.Constraints(1.0, 1.0));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 2.4);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0.0);

    public Module(ModuleIO io) {
        this.io = io;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.wheelVelocityMetersPerSec,
                new Rotation2d(inputs.azimuthEncoderPositionRads));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
    }

    private void updateFromDesiredState() {
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

        double wheelVolts = MathUtil.clamp(driveOutput + driveFeedforward, -12, 12);
        double azimuthVolts = MathUtil.clamp(turnOutput + turnFeedforward, -12, 12);

        Logger.getInstance().recordOutput("Module (" + io.getConfig().getName() + ") wheel volts", wheelVolts);
        Logger.getInstance().recordOutput("Module (" + io.getConfig().getName() + ") azimuth volts", azimuthVolts);

        io.setWheelVolts(wheelVolts);
        io.setAzimuthVolts(azimuthVolts);
    }

    public ModuleConfig getConfig() {
        return io.getConfig();
    }

    public void periodic() {
        io.updateInputs(inputs);

        updateFromDesiredState();
        SmartDashboard.putNumber("Wheel P", inputs.wheelPositionMeters);
        SmartDashboard.putNumber("Wheel V", inputs.wheelVelocityMetersPerSec);
        SmartDashboard.putNumber("Azimuth P", inputs.azimuthEncoderPositionRads);

        Logger.getInstance().processInputs("Module (" + io.getConfig().getName() + ")", inputs);
    }
}
