package frc.robot;

import frc.robot.Component.BallIntake;
import frc.robot.Component.BallThrower;
import frc.robot.Component.ColorSensor;
import frc.robot.Component.PneumaticSystem;
import frc.robot.Component.Swerve;
import frc.robot.Component.Data.WheelData;
import frc.robot.Math.PID;
import frc.robot.Math.Timer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class Robot extends TimedRobot {

  private final ColorSensor _ColorSensor = new ColorSensor(7);
  private final PneumaticSystem _PneumaticSystem = new PneumaticSystem();

  public static Swerve SwerveDrive;
  public static BallThrower Thrower;
  public static BallIntake Intake;

  public static PID DirectionHoldPID = new PID(3, 0, 0);
  public static PID PositionHoldPID = new PID(0, 0, 0);

  @Override
  public void robotInit() {
    //Initializing the SwerveDrive drivetrain
    WheelData[] data = {
      new WheelData(15, 7, 3, 3, new Vector2d(1, 1), 3.6768434- (3.1415/2.0)),
      new WheelData(14, 8, 0, 0, new Vector2d(1, -1), 4.331834+ (3.1415/2.0)),
      new WheelData(17, 9, 1, 1, new Vector2d(-1, -1), 5.00063+ (3.1415/2.0)),
      new WheelData(18, 4, 2, 2, new Vector2d(-1, 1), 4.387056- (3.1415/2.0))
    };

    SwerveDrive = new Swerve(data, new Joystick(0));
    SwerveDrive.SetCurrentMode(Swerve.DrivingMode.World);

    SwerveDrive.SetPIDGain(0, 1, 0, 0);
    SwerveDrive.SetPIDGain(1, 1, 0, 0);
    SwerveDrive.SetPIDGain(2, 1, 0, 0);
    SwerveDrive.SetPIDGain(3, 1, 0, 0);

    SwerveDrive.SetDeadzone(0.2);
    SwerveDrive.InitIMU();

    Thrower = new BallThrower(SwerveDrive, 4, 2);
    Intake = new BallIntake(0.25, 0, 0, 0);
  }

  
  @Override
  public void teleopInit() {
    super.teleopInit();

    Thrower.Init();
    Intake.InitIntake();
    SwerveDrive.RecalibrateIMU();
    Timer.Init();
  }

  @Override
  public void teleopPeriodic() { 
    //Calculate the first time after use GetDeltaTime()
    Timer.Calculate();

    //Execute Needed Component
    _PneumaticSystem.CheckPressure();
    
    Thrower.DoThrower();
    Intake.DoIntake();
    SwerveDrive.DoSwerve();
    //_ColorSensor.GetColor();
  } 
}