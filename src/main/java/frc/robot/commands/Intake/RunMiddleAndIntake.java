// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RunMiddleAndIntake extends CommandBase {
  /** Input true to reverse the intake, false if not. */
  boolean r;
  boolean stopOnLimit;
  public RunMiddleAndIntake(boolean reverse) {
    addRequirements(Robot.intake);
    r = reverse;
    stopOnLimit = true;
  }

  public RunMiddleAndIntake() {
    addRequirements(Robot.intake);
    stopOnLimit = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.intake.runFullIntake(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.runFullIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stopOnLimit == false){
      return false;
    } else {
      return Robot.intake.isMiddleLimitActivated();
    }   
  }
}
