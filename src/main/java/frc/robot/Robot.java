package frc.robot;

import frc.robot.Component.Autonomous;
import frc.robot.Component.BallIntake;
import frc.robot.Component.BallThrower;
import frc.robot.Component.Climber;
import frc.robot.Component.Leds;
import frc.robot.Component.PneumaticSystem;
import frc.robot.Component.Swerve;
import frc.robot.Component.Autonomous.AutonomousMode;
import frc.robot.Component.Data.InputManager;
import frc.robot.Component.Data.LimeLightPosition;
import frc.robot.Component.Data.RobotOdometry;
import frc.robot.Component.Data.WheelData;
import frc.robot.Component.Swerve.ShifterMode;
import frc.robot.Math.PID;
import frc.robot.Math.Timer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  public static Swerve SwerveDrive;
  public static RobotOdometry Odometry;

  public static BallThrower Thrower;
  public static BallIntake Intake;

  public static Climber Climber;

  public static Leds Leds;

  public static PID DirectionHoldPID = new PID(0.000523599, 0.0010472, 0);
  public static PID PositionHoldPID = new PID(0.3, 0, 0);


  public static LimeLightPosition Position = new LimeLightPosition(new Vector2d(0, 0));

  private SendableChooser<AutonomousMode> _autoChooser = new SendableChooser<AutonomousMode>();
  private Autonomous _auto;
  
  private boolean _isInit = false;

  public static void Println(Object Line)
  {
    System.out.print(Line);
  }

  @Override
  public void robotInit() {
    //Initializing the SwerveDrive drivetrain
    WheelData[] data = {
      new WheelData(20, 7, new Vector2d(0, 1), 0, 3, new Vector2d(-10, -11), 0.6964067- (3.1415/2.0)),
      new WheelData(21, 8, new Vector2d(4, 5), 1, 0, new Vector2d(10, -11), 4.2- (3.1415/2.0)),
      new WheelData(22, 9, new Vector2d(2, 3), 2, 1, new Vector2d(10, 11), 0.8344609- (3.1415/2.0)),
      new WheelData(23, 4, new Vector2d(6, 7), 3, 2, new Vector2d(-10, 11), 3.1476357- (3.1415/2.0))
    };

    SwerveDrive = new Swerve(data);

    SwerveDrive.SetShifterMode(ShifterMode.Manual);
    SwerveDrive.SetShiftThreshold(90. * 4., 230. * 4.);
    SwerveDrive.SetShiftMinTime(1.5);

    SwerveDrive.SetCurrentMode(Swerve.DrivingMode.World);

    SwerveDrive.SetPIDGain(0, 1, 0, 0);
    SwerveDrive.SetPIDGain(1, 1, 0, 0);
    SwerveDrive.SetPIDGain(2, 1, 0, 0);
    SwerveDrive.SetPIDGain(3, 1, 0, 0);

    SwerveDrive.SetRateLimiter(5);
    SwerveDrive.SetRotationRateLimiter(10);

    SwerveDrive.InitIMU();

    Thrower = new BallThrower(500, 4000);
    Intake = new BallIntake(6, 6, 7, 2, 3, -1200);

    Leds = new Leds(0, 1, 2);

    Climber = new Climber(14);

    InputManager.Init();

    for(AutonomousMode mode : AutonomousMode.values())
    {
      _autoChooser.addOption(mode.toString(), mode);
    }

    SmartDashboard.putData("AutonomousSelector", _autoChooser);
  }
  
  @Override
  public void autonomousInit() 
  {
    Init();

    _auto = new Autonomous(_autoChooser.getSelected());
    _auto.Init();
  }

  @Override
  public void autonomousPeriodic() 
  {
    Timer.Calculate();

    _auto.DoSystem();
  }

  @Override
  public void teleopInit() 
  {
    if(!_isInit)
    {
      Init();
    }

    Thrower.SetAutoShoot(false);
    Thrower.StopOverrideAlign();
    _isInit = false; //Put it to false so it init next time you enable the robot
  }

  public void Init()
  {
    //Initialize all the robot component before a match
    SwerveDrive.RecalibrateIMU();

    Thrower.Init();
    Intake.Init();
    
    Climber.Init();

    PneumaticSystem.Init();
    Timer.Init();

    _isInit = true;

    //System.out.println(Position.GetCurrent().x + " " + Position.GetCurrent().x);
  }

  @Override
  public void teleopPeriodic() { 
    //Calculate the first time after use GetDeltaTime()
    Timer.Calculate();
    InputManager.DoInputManager();

    //Execute Needed Component
    PneumaticSystem.CheckPressure();
    
    Thrower.DoSystem();
    Intake.DoSystem();

    Climber.DoSystem();

    Leds.DoSystem();
    Leds.SetColor("off");

    SwerveDrive.DoSystem();
  } 
}