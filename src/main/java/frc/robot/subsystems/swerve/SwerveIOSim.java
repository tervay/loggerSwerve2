package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleContainer;

public class SwerveIOSim implements SwerveIO {

    AnalogGyro gyro = new AnalogGyro(0);
    AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

    Translation2d frontLeft = new Translation2d();
    Translation2d frontRight = new Translation2d();

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.gyroHeadingRad = Units.degreesToRadians(gyro.getAngle());
    }

    @Override
    public void resetGyro(Rotation2d rot) {
        // gyro.reset();
        gyroSim.setAngle(rot.getDegrees());
    }

    public void updateSimGyro(ModuleContainer modules) {

    }
}
