package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.commandgroups.AutoShootCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveCommandSetValue;
import frc.robot.commands.Sync_Encoder;
import frc.robot.commands.Turn_to_Angle_Command;
import frc.robot.commands.Turn_to_Angle_New;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.IntakeRunVariableSpeed;
import frc.robot.commands.Intake.RunMiddleAndIntake;
import frc.robot.commands.Shoot.AutoShootGroup;
import frc.robot.commands.Shoot.LoadShooterCommand;
import frc.robot.commands.Shoot.ShootCommand;
import frc.robot.commands.climber.ExtendRetractClimber;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PID_DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Timer;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
import frc.robot.subsystems.ShooterSubsystem;
public class Robot extends TimedRobot {
  /*
   * Put subsystems Create object called lift defined by rules of lift subsystems
   */
  // public static DrivetrainSubsystem drive;
  public static PID_DrivetrainSubsystem drive;
  public static ShooterSubsystem shooter;
  public static IntakeSubsystem intake;
  public static OI m_oi;
  public static ClimberSubsystem climber;
  Timer timer = new Timer();
  // private Ultrasonic sonic = new Ultrasonic(4, 4);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // new HomeCommandGroup().start();
    // CameraServer.startAutomaticCapture(0);
    // shooter = new (subsystem here)
    drive = new PID_DrivetrainSubsystem();
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    climber = new ClimberSubsystem();
    m_oi = new OI();
    CommandScheduler.getInstance().setDefaultCommand(drive, new DriveCommand(drive));
    SmartDashboard.putNumber("Shooter Speed Testing", 0);
    SmartDashboard.putNumber("Shooter Angle", 0);
    SmartDashboard.putBoolean("Shooter Done?", false);
    SmartDashboard.putNumber("MiddleIntakeSpeed", 0);
    SmartDashboard.putNumber("MiddleSpeed", 0);
    SmartDashboard.putNumber("JustIntakeSpeed", 0);
    SmartDashboard.putNumber("Climber Goto", 0);
    SmartDashboard.putNumber("turnkP", 0);
    SmartDashboard.putNumber("turnkI", 0);
    SmartDashboard.putNumber("turnkD", 0);
    
    //SmartDashboard.putNumber("Distance From Goal to Limelight", distanceFromLimelightToGoalInches);
    SmartDashboard.putNumber("Angle off from Goal", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    SmartDashboard.putNumber("Tv", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
    //SmartDashboard.putNumber("Camera", 1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Forward", m_oi.getY());
    SmartDashboard.putNumber("Strafe", -m_oi.getX());
    SmartDashboard.putNumber("Rotate", -m_oi.getTwist());

    SmartDashboard.putNumber("FR Angle", drive.frontRight.getEncoder());
    SmartDashboard.putNumber("FL Angle", drive.frontLeft.getEncoder());
    SmartDashboard.putNumber("BL Angle", drive.backLeft.getEncoder());
    SmartDashboard.putNumber("BR Angle", drive.backRight.getEncoder());

    SmartDashboard.putNumber("FR offset", drive.frontRight.getOffset());
    SmartDashboard.putNumber("FL offset", drive.frontLeft.getOffset());
    SmartDashboard.putNumber("BL offset", drive.backLeft.getOffset());
    SmartDashboard.putNumber("BR offset", drive.backRight.getOffset());

    SmartDashboard.putNumber("FR ABS Encoder", drive.frontRight.getRawAbsEncoder());
    SmartDashboard.putNumber("FL ABS Encoder", drive.frontLeft.getRawAbsEncoder());
    SmartDashboard.putNumber("BL ABS Encoder", drive.backLeft.getRawAbsEncoder());
    SmartDashboard.putNumber("BR ABS Encoder", drive.backRight.getRawAbsEncoder());

    SmartDashboard.putNumber("FR ABS Encoder Adjusted", drive.frontRight.getAdjustedAbsEncoder());
    SmartDashboard.putNumber("FL ABS Encoder Adjusted", drive.frontLeft.getAdjustedAbsEncoder());
    SmartDashboard.putNumber("BL ABS Encoder Adjusted", drive.backLeft.getAdjustedAbsEncoder());
    SmartDashboard.putNumber("BR ABS Encoder Adjusted", drive.backRight.getAdjustedAbsEncoder());

    SmartDashboard.putNumber("AHS Angle", drive.getAngle());

    SmartDashboard.putNumber("Current Shooter Angle", shooter.shooterencoder());
    SmartDashboard.putNumber("Climb Encoder Value", climber.getEncoder());
    SmartDashboard.putBoolean("Is Limit Switch Pressed?", climber.checkIfAtLimit());
    SmartDashboard.putBoolean("feeder Limit", shooter.feederLimitCheck());
    SmartDashboard.putBoolean("front limit", intake.isMiddleLimitActivated());

    SmartDashboard.putNumber("Gyro Displacement X", drive.getXDisplacement());
    SmartDashboard.putNumber("Gyro Dispalcement Y", drive.getYDisplacement());
    SmartDashboard.putNumber("Gyro Dispalcement Z", drive.getZDisplacement());

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);

    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    SmartDashboard.putNumber("ty", NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));

    // distance from the target to the floor
    double angleToGoalDegrees = RobotMap.limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (RobotMap.goalHeightInches - RobotMap.limelightLensHeightInches)/Math.tan(angleToGoalRadians);


    SmartDashboard.putNumber("Target Distance", distanceFromLimelightToGoalInches);
    SmartDashboard.putNumber("Angle off from Goal", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    SmartDashboard.putNumber("Tv", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
    // SmartDashboard.putNumber("Left Leg Encoder", legs.getLeftLeg());
    // SmartDashboard.putNumber("Right Leg Encoder", legs.getRightLeg());

    // SmartDashboard.putNumber("Camera Position", cam1.getLocation());

    // SmartDashboard.putBoolean("stowLimit", lift.stowLimit.get());
    // SmartDashboard.putBoolean("lowLimit", lift.lowLimit.get());
    // SmartDashboard.putBoolean("middleLimit", lift.middleLimit.get());
    // SmartDashboard.putBoolean("highLimit", lift.highLimit.get());
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    Robot.drive.angleReset();
    CommandScheduler.getInstance().schedule(new Sync_Encoder());
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (timer.get() < 2) {
      CommandScheduler.getInstance().schedule(new ExtendIntake(), new ExtendRetractClimber(false));
    } else if (timer.get() < 8.4) {
      CommandScheduler.getInstance().schedule(new RunMiddleAndIntake());
      if (timer.get() < 5) {
        CommandScheduler.getInstance().schedule(new DriveCommandSetValue(-0.25, 0, 0, 0.8));
      } else if (timer.get() < 8.3) {
        CommandScheduler.getInstance().schedule(true, new Turn_to_Angle_New(180, Robot.drive));
      } 
    } else if (timer.get() < 15) {
      CommandScheduler.getInstance().schedule(new AutoShootGroup());
    }
    CommandScheduler.getInstance().run();
  }
 
  @Override
  public void teleopInit() {
    // new HomeCommandGroup().start();
    new Sync_Encoder();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
