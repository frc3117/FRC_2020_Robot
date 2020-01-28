package frc.robot;

import frc.robot.Component.AutonomousSequence;
import frc.robot.Component.BallIntake;
import frc.robot.Component.BallThrower;
import frc.robot.Component.ColorSensor;
import frc.robot.Component.PneumaticSystem;
import frc.robot.Component.Swerve;
import frc.robot.Component.Data.AutonomousSequenceAction;
import frc.robot.Component.Data.RobotOdometry;
import frc.robot.Component.Data.WheelData;
import frc.robot.Math.PID;
import frc.robot.Math.Timer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class Robot extends TimedRobot {

  private final ColorSensor _ColorSensor = new ColorSensor(0, 7);

  public static Swerve SwerveDrive;
  public static BallThrower Thrower;
  public static BallIntake Intake;

  public static RobotOdometry Odometry;

  public static PID DirectionHoldPID = new PID(3, 0, 0);
  public static PID PositionHoldPID = new PID(0.3, 0, 0);

  @Override
  public void robotInit() {
    //Initializing the SwerveDrive drivetrain
    WheelData[] data = {
      new WheelData(15, 7, new Vector2d(0, 1), 3, 3, new Vector2d(1, 1), 3.6768434- (3.1415/2.0)),
      new WheelData(14, 8, new Vector2d(4, 5), 0, 0, new Vector2d(1, -1), 4.331834+ (3.1415/2.0)),
      new WheelData(17, 9, new Vector2d(2, 3), 1, 1, new Vector2d(-1, -1), 5.00063+ (3.1415/2.0)),
      new WheelData(18, 4, new Vector2d(6, 7), 2, 2, new Vector2d(-1, 1), 4.387056- (3.1415/2.0))
    };

    SwerveDrive = new Swerve(data, new Joystick(0));
    SwerveDrive.SetCurrentMode(Swerve.DrivingMode.World);

    SwerveDrive.SetPIDGain(0, 1, 0, 0);
    SwerveDrive.SetPIDGain(1, 1, 0, 0);
    SwerveDrive.SetPIDGain(2, 1, 0, 0);
    SwerveDrive.SetPIDGain(3, 1, 0, 0);

    SwerveDrive.SetDeadzone(0.2);
    SwerveDrive.InitIMU();

    Thrower = new BallThrower(SwerveDrive, 4, 2, 500, 3000);
    Intake = new BallIntake(0, 0, 0);

    Odometry = new RobotOdometry(0.05, 1.2192, 6.7056);
  }

  
  @Override
  public void teleopInit() {
    super.teleopInit();

    SwerveDrive.RecalibrateIMU();
    Thrower.Init();
    Intake.Init();

    PneumaticSystem.Init();
    Timer.Init();

    Odometry.SetPosition(new Vector2d(0, 0));

    AutonomousSequence autonomousSequence = new AutonomousSequence(new Vector2d(0, 0), 
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 1)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 0)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 1)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 0)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 1)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 0)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 1)),
    AutonomousSequenceAction.CreateMoveTo(5, new Vector2d(0, 0))
    );

    autonomousSequence.StartSequence();
  }

  @Override
  public void teleopPeriodic() { 
    //Calculate the first time after use GetDeltaTime()
    Timer.Calculate();

    //Execute Needed Component
    PneumaticSystem.CheckPressure();
    
    Thrower.DoThrower();
    Intake.DoIntake();

    SwerveDrive.DoSwerve();
    Odometry.DoOdometry();

    //System.out.println(Odometry.GetPosition().x + " : " + Odometry.GetPosition().y);
    //_ColorSensor.DoColorSensor();;
  } 
}