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
  public RunMiddleAndIntake() {
    addRequirements(Robot.intake);
    addRequirements(Robot.shooter);
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
    if (Robot.shooter.feederLimitCheck() == false) {
      Robot.shooter.feederSpeed(0.5);
    }  
    if (Robot.intake.isMiddleLimitActivated() == true) {
      Robot.intake.intakeSpeed(.75);
      Robot.intake.runMiddleMotor(-1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.shooter.feederLimitCheck() == true) {
      Robot.shooter.feederSpeed(0);
      if (Robot.intake.isMiddleLimitActivated() == false) {
        Robot.intake.runMiddleMotor(0);
        Robot.intake.intakeSpeed(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.intake.intakeStop();
    Robot.intake.runMiddleMotor(0);
    Robot.shooter.feederSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!Robot.intake.isMiddleLimitActivated() && Robot.shooter.feederLimitCheck());
  }
}
