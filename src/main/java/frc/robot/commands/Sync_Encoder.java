// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Sync_Encoder extends CommandBase {
  /** Creates a new Sync_Encoder. */
  public Sync_Encoder() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drive.frontLeft.setEncoder((Robot.drive.frontLeft.getAdjustedAbsEncoder()/2.5)-1);
    Robot.drive.frontRight.setEncoder((Robot.drive.frontRight.getAdjustedAbsEncoder()/2.5)-1);
    Robot.drive.backLeft.setEncoder((Robot.drive.backLeft.getAdjustedAbsEncoder()/2.5)-1);
    Robot.drive.backRight.setEncoder((Robot.drive.backRight.getAdjustedAbsEncoder()/2.5)-1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
