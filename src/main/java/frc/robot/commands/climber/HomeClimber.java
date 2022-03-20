// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class HomeClimber extends CommandBase {
  /** Creates a new HomeClimber. */
  public HomeClimber() {
    addRequirements(Robot.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.climber.pullWinch(-0.20);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.climber.resetEncoder();
    Robot.climber.pullWinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.climber.checkIfAtLimit();
  }
}
