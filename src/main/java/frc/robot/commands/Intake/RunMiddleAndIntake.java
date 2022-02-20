// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RunMiddleAndIntake extends CommandBase {
  
  boolean r;
  boolean stopOnLimit;
  boolean d;
  double s;

  /**
   * 
   * @param speed speed if not pulling from dash, -1 to 1
   * @param reverse if reversing intake
   * @param fromDashboard true if you are pulling from smartdash
   * @param stopOnLimits whether to stop on the intake limitswitch
   */
  public RunMiddleAndIntake(double speed, boolean reverse, boolean fromDashboard, boolean stopOnLimits) {
    addRequirements(Robot.intake);
    r = reverse;
    stopOnLimit = stopOnLimits;
    d = fromDashboard;
    s = speed;
  }

  /*
   * public RunMiddleAndIntake(double speed) {
   * addRequirements(Robot.intake);
   * stopOnLimit = false;
   * s = speed;
   * }
   */
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (d == false) {
      Robot.intake.runFullIntake(s);
    } else {
      Robot.intake.runFullIntake(SmartDashboard.getNumber("MiddleIntakeSpeed", 0));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.runFullIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stopOnLimit == false) {
      return false;
    } else {
      return Robot.intake.isMiddleLimitActivated();
    }
  }
}
