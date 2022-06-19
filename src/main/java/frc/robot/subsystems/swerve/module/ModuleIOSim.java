package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public class ModuleIOSim implements ModuleIO {

    ModuleConfig config;

    FlywheelSim azimuthSim, wheelSim;

    double wheelLastVelocity;

    public ModuleIOSim(ModuleConfig config) {
        this.config = config;
        azimuthSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0,
                (1.0 / 12.0) * (Units.lbsToKilograms(120.0) / 4) * Units.inchesToMeters(1.5 * 1.5));
        wheelSim = new FlywheelSim(DCMotor.getNEO(1), 6.12,
                (1.0 / 12.0) * (Units.lbsToKilograms(120.0) / 4) * Units.inchesToMeters(2 * 2));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        for (int i = 0; i < 20; i++) {
            wheelSim.update(0.001);
            azimuthSim.update(0.001);
        }

        inputs.wheelPositionMeters += wheelSim.getAngularVelocityRPM()
                * Math.PI
                * Units.inchesToMeters(4)
                * 0.02
                / 60;

        inputs.wheelVelocityMetersPerSec = wheelSim.getAngularVelocityRPM() * Math.PI * Units.inchesToMeters(4) / 60;
        inputs.azimuthEncoderPositionDeg += Units.radiansToDegrees(azimuthSim.getAngularVelocityRadPerSec()) * 0.02;
        inputs.azimuthEncoderVelocityDegPerSec = Units.radiansToDegrees(azimuthSim.getAngularVelocityRadPerSec());
    }

    @Override
    public void setWheelVolts(double volts) {
        wheelSim.setInputVoltage(volts);
    }

    @Override
    public void setAzimuthVolts(double volts) {
        azimuthSim.setInputVoltage(volts);
    }

    @Override
    public ModuleConfig getConfig() {
        return config;
    }

}
