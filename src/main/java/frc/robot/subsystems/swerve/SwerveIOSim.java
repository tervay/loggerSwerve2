package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleContainer;

public class SwerveIOSim implements SwerveIO {

    AnalogGyro gyro = new AnalogGyro(0);
    AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.gyroHeadingRad = gyro.getRotation2d().getRadians();
    }

    @Override
    public void resetGyro(Rotation2d rot) {
        gyro.reset();
    }


    public void updateSimGyro(ModuleContainer modules) {
        for (Module m : new Module[] { modules.getFrontLeft() }) {
        }

    }
}
