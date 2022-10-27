// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Swerve;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  Swerve swerve;
  XboxController driver;
  PIDController snapController = new PIDController(1, 0, 0);

  boolean isSnapping;

  public Drive(Swerve swerve, XboxController driver) {
    this.swerve = swerve;
    this.driver = driver;
    isSnapping = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (driver.getPOV() != -1) {
      snapController.setSetpoint(driver.getPOV());
      isSnapping = true;
    }

    if (isSnapping) {
      // swerve.drive(driver.getLeftX(), driver.getLeftY(),
      //     snapController.calculate(swerve.getPose().getRotation().getDegrees()));
    } else {
      // swerve.drive(driver.getLeftX(), driver.getLeftY(), driver.getRightTriggerAxis());
    }

    if (driver.getRightTriggerAxis() != 0) {
      isSnapping = false;
    }

    Logger.getInstance().recordOutput("Wooo!", isSnapping);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
