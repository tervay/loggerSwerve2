// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.module.ModuleConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static final boolean tuningMode = true;
        private static final double kSquareChassisSize = Units.inchesToMeters(26);
        public static final double kMaxTranslationalSpeedMetersPerSec = Units.feetToMeters(16);
        public static final double kMaxRotationalSpeedDegPerSec = 360;

        private static final PIDFFGains kDefaultAzimuthGains = PIDFFGains.builder("Default Azimuth").kP(0.08).build();
        private static final PIDFFGains kDefaultDrivingGains = PIDFFGains.builder("Default Driving").kP(12.0).kV(2.45)
                        .build();

        public static final ModuleConfig kFrontLeftModuleConfig = ModuleConfig.builder()
                        .name("Front Left")
                        .driveMotorCANId(1)
                        .azimuthMotorCANid(2)
                        .drivingGains(kDefaultDrivingGains)
                        .azimuthGains(kDefaultAzimuthGains)
                        .encoderOffset(1.0)
                        .location(new Translation2d(kSquareChassisSize / 2.0, kSquareChassisSize / 2.0))
                        .build();
        public static final ModuleConfig kFrontRightModuleConfig = ModuleConfig.builder()
                        .name("Front Right")
                        .driveMotorCANId(3)
                        .azimuthMotorCANid(4)
                        .drivingGains(kDefaultDrivingGains)
                        .azimuthGains(kDefaultAzimuthGains)
                        .encoderOffset(1.0)
                        .location(new Translation2d(kSquareChassisSize / 2.0, -kSquareChassisSize / 2.0))
                        .build();
        public static final ModuleConfig kBackRightModuleConfig = ModuleConfig.builder()
                        .name("Back Right")
                        .driveMotorCANId(5)
                        .azimuthMotorCANid(6)
                        .drivingGains(kDefaultDrivingGains)
                        .azimuthGains(kDefaultAzimuthGains)
                        .encoderOffset(1.0)
                        .location(new Translation2d(-kSquareChassisSize / 2.0, kSquareChassisSize / 2.0))
                        .build();
        public static final ModuleConfig kBackLeftModuleConfig = ModuleConfig.builder()
                        .name("Back Left")
                        .driveMotorCANId(7)
                        .azimuthMotorCANid(8)
                        .drivingGains(kDefaultDrivingGains)
                        .azimuthGains(kDefaultAzimuthGains)
                        .encoderOffset(1.0)
                        .location(new Translation2d(-kSquareChassisSize / 2.0, -kSquareChassisSize / 2.0))
                        .build();

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
                        kFrontLeftModuleConfig.getLocation(),
                        kFrontRightModuleConfig.getLocation(),
                        kBackLeftModuleConfig.getLocation(),
                        kBackRightModuleConfig.getLocation());
}
