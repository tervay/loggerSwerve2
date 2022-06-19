package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class SimmableGyro {
    ADXRS450_Gyro realGyro;

    boolean isReal;

    public static double[] gyro_rates = new double[5];
    public static double[] gyro_angles = new double[5];

    public SimmableGyro() {
        isReal = Robot.isReal();

        if (isReal) {
            realGyro = new ADXRS450_Gyro();
        } else {
            realGyro = null;
        }
    }

    public void reset() {
        if (isReal) {
            realGyro.reset();
        } else {
            gyro_rates[0] = 0;
            gyro_angles[0] = 0;
        }

    }

    public void calibrate() {
        if (isReal) {
            realGyro.calibrate();
        } else {
            // nada;
        }

    }

    public double getRate() {
        if (isReal) {
            return realGyro.getRate();
        } else {
            return gyro_rates[0];
        }
    }

    public double getAngle() {
        if (isReal) {
            return realGyro.getAngle();
        } else {
            return gyro_angles[0];
        }
    }

    public boolean isConnected() {
        if (isReal) {
            return realGyro.isConnected();
        } else {
            return true;
        }
    }

    public void simUpdate(double newRate, double sampleTime) {
        gyro_rates[0] = newRate;
        gyro_angles[0] += newRate * sampleTime;
    }

    public void simSetAngle(double newAngle) {
        gyro_rates[0] = 0;
        gyro_angles[0] = newAngle;
    }

}
