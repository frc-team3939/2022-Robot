// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class IntakeRunVariableSpeed extends CommandBase {
  double speed;
  boolean d;
  public IntakeRunVariableSpeed(double intakeSpeed, boolean fromDash) {
    addRequirements(Robot.intake);
    speed = intakeSpeed;
    d = fromDash;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (d == false){
      Robot.intake.intakeSpeed(speed);
    } else {
      Robot.intake.intakeSpeed(SmartDashboard.getNumber("JustIntakeSpeed", 0));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
