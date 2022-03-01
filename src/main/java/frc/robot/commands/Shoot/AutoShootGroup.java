// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shoot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.TurnToVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootGroup extends SequentialCommandGroup {
  /** Creates a new AutoShootGroup. */
  double a;
  public AutoShootGroup(double angle) {
    // Add your commands in the addCommands() call, e.g.
    /*double distance = SmartDashboard.getNumber("Target Distance", 100);
    NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    SmartDashboard.putNumber("tyinstant", targetOffsetAngle_Vertical);

    // distance from the target to the floor
    double angleToGoalDegrees = RobotMap.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (RobotMap.goalHeightInches - RobotMap.limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    */
    addCommands(new TurnToVision(Robot.drive), new adjustAngleCommand(((4.9779 * a) + 144.57))  , new ShootCommand(.000805*SmartDashboard.getNumber("Target Distance", 0) + .50058));
  }
}
