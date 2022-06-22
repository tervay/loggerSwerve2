// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Function;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.LogSocketServer;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIOSim;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleContainer;
import frc.robot.subsystems.swerve.module.ModuleIOSim;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  Swerve swerve;

  PIDController xController = new PIDController(1.6, 0, 0);
  PIDController yController = new PIDController(1.5, 0, 0);
  ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(Math.PI, Math.PI));
  HolonomicDriveController holonomicDriveController = new HolonomicDriveController(xController, yController,
      thetaController);

  Module module;
  Timer timer = new Timer();

  // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
  // List.of(new Pose2d(0, 0, new Rotation2d()),
  // new Pose2d(2, 5, Rotation2d.fromDegrees(0)),
  // new Pose2d(5, 5,
  // Rotation2d.fromDegrees(0)),
  // new Pose2d(1, 1,
  // Rotation2d.fromDegrees(0))

  // ),
  // new TrajectoryConfig(Units.feetToMeters(15), Units.feetToMeters(15)));

  // Trajectory trajectory = PathPlanner.
  private Command m_autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // swerve = new Swerve(
    // new SwerveIOPigeon2(),
    // ModuleContainer.builder()
    // .frontLeft(new Module(new ModuleIOMk4iNEO(Constants.kFrontLeftModuleConfig)))
    // .frontRight(new Module(new
    // ModuleIOMk4iNEO(Constants.kFrontRightModuleConfig)))
    // .backLeft(new Module(new ModuleIOMk4iNEO(Constants.kBackLeftModuleConfig)))
    // .backRight(new Module(new ModuleIOMk4iNEO(Constants.kBackRightModuleConfig)))
    // .build());

    LoggedNetworkTables.getInstance().addTable("/SmartDashboard");

    if (isReal()) {
      Logger.getInstance().addDataReceiver(new ByteLogReceiver("logs/"));
    }

    Logger.getInstance().addDataReceiver(new LogSocketServer(5800));
    Logger.getInstance().start();

    // module = new Module(new ModuleIOSim(Constants.kFrontLeftModuleConfig));

    swerve = new Swerve(
        new SwerveIOSim(),
        ModuleContainer.builder()
            .frontLeft(new Module(new ModuleIOSim(Constants.kFrontLeftModuleConfig)))
            .frontRight(new Module(new ModuleIOSim(Constants.kFrontRightModuleConfig)))
            .backLeft(new Module(new ModuleIOSim(Constants.kBackLeftModuleConfig)))
            .backRight(new Module(new ModuleIOSim(Constants.kBackRightModuleConfig)))
            .build());

    // System.out.println(trajectory.getTotalTimeSeconds());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    swerve.drive(0, 0, 0);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    timer.start();

    // m_autonomousCommand = new RunCommand(() -> {
    // ChassisSpeeds speeds = holonomicDriveController.calculate(
    // swerve.getPose(),
    // trajectory.sample(timer.get()),
    // Rotation2d.fromDegrees(45));

    // if (timer.get() > trajectory.getTotalTimeSeconds()) {
    // swerve.drive(0, 0, 0);
    // } else {
    // swerve.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
    // speeds.omegaRadiansPerSecond);
    // }
    // }, swerve);

    var t = PathPlanner.loadPath("xTune", 4.87, 4.5);
    swerve.setPose(t.getInitialPose());

    m_autonomousCommand = new PPSwerveControllerCommand(
        t, () -> swerve.getPose(), swerve.getKinematics(), xController, yController,
        thetaController, (states) -> {
          swerve.setStates(states);
        }, swerve).andThen(() -> swerve.drive(0, 0, 0));

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    swerve.drive(0, 0, 0);
    // module.setDesiredState(new SwerveModuleState(3, Rotation2d.fromDegrees(45)));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    Function<Module, Double> f = (module) -> {
      return Math.abs(module.getInputs().wheelCurrentAmps) + Math.abs(module.getInputs().azimuthCurrentAmps);
    };

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            f.apply(swerve.getModules().getFrontLeft()),
            f.apply(swerve.getModules().getFrontRight()),
            f.apply(swerve.getModules().getBackLeft()),
            f.apply(swerve.getModules().getBackRight())));
  }
}
