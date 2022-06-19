package frc.robot.subsystems.swerve.module;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.util.Units;

public class ModuleIOMk4iNEO implements ModuleIO {

    ModuleConfig config;
    CANSparkMax wheelMotor, azimuthMotor;
    SparkMaxAnalogSensor azimuthEncoder;

    public ModuleIOMk4iNEO(ModuleConfig config) {
        this.config = config;

        wheelMotor = new CANSparkMax(config.getDriveMotorCANId(), MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(config.getAzimuthMotorCANid(), MotorType.kBrushless);

        wheelMotor.restoreFactoryDefaults();
        azimuthMotor.restoreFactoryDefaults();

        azimuthEncoder = azimuthMotor.getAnalog(Mode.kAbsolute);
        azimuthMotor.getPIDController().setFeedbackDevice(azimuthEncoder);

        wheelMotor.getEncoder().setPositionConversionFactor((1.0 / 6.12) * Units.inchesToMeters(4) * Math.PI);
        wheelMotor.getEncoder().setVelocityConversionFactor((1.0 / 6.12) * Units.inchesToMeters(4) * Math.PI / 60.0);

        azimuthEncoder.setPositionConversionFactor(7.0 / 150.0);
        azimuthEncoder.setVelocityConversionFactor((7.0 / 150.0) / 60.0);

        wheelMotor.setSmartCurrentLimit(50);
        azimuthMotor.setSmartCurrentLimit(50);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.azimuthEncoderPositionDeg = azimuthEncoder.getPosition();
        inputs.wheelPositionMeters = wheelMotor.getEncoder().getPosition();
    }

    @Override
    public void setWheelVolts(double volts) {
        wheelMotor.setVoltage(volts);
    }

    @Override
    public void setAzimuthVolts(double volts) {
        azimuthMotor.setVoltage(volts);
    }

    @Override
    public ModuleConfig getConfig() {
        return config;
    }

}
