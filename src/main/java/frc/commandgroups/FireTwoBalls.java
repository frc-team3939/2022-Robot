// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake.RunMiddle;
import frc.robot.commands.Shoot.ShooterSpeedCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireTwoBalls extends ParallelCommandGroup {
  /**
   * In matches autoshoot would ALWAYS take preference - for testing purposes.
   */
  public FireTwoBalls() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShooterSpeedCommand(0.3, 0.7, false), new RunMiddle(1, false, 75, false));
  }
}
