/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Component.BallThrower;
import frc.robot.Component.ColorSensor;
import frc.robot.Component.PneumaticSystem;
import frc.robot.Component.Swerve;
import frc.robot.Component.Data.WheelData;
import frc.robot.Math.Mathf;
import frc.robot.Math.PID;
import frc.robot.Math.Polar;
import frc.robot.Math.Timer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class Robot extends TimedRobot {

  private final ColorSensor _ColorSensor = new ColorSensor(7);
  private final PneumaticSystem _PneumaticSystem = new PneumaticSystem();

  private Swerve _swerve;
  private BallThrower _thrower;

  private PID _directionHoldPID = new PID(3, 0, 0);

  @Override
  public void robotInit() {
    
    //Initializing the swerve drivetrain
    WheelData[] data = {
      new WheelData(15, 7, 3, 3, new Vector2d(1, 1), 3.6768434- (3.1415/2.0)),
      new WheelData(14, 8, 0, 0, new Vector2d(1, -1), 4.331834+ (3.1415/2.0)),
      new WheelData(17, 9, 1, 1, new Vector2d(-1, -1), 5.00063+ (3.1415/2.0)),
      new WheelData(18, 4, 2, 2, new Vector2d(-1, 1), 4.387056- (3.1415/2.0))
    };

    _swerve = new Swerve(data, new Joystick(0));
    _swerve.SetCurrentMode(Swerve.DrivingMode.World);

    _swerve.SetPIDGain(0, 1, 0, 0);
    _swerve.SetPIDGain(1, 1, 0, 0);
    _swerve.SetPIDGain(2, 1, 0, 0);
    _swerve.SetPIDGain(3, 1, 0, 0);

    _swerve.SetDeadzone(0.2);
    _swerve.InitIMU();

    _thrower = new BallThrower(_swerve, 4, 2);
  }

  
  @Override
  public void teleopInit() {
    super.teleopInit();

    _thrower.Init();
    _swerve.RecalibrateIMU();
    Timer.Init();
  }

  @Override
  public void teleopPeriodic() { 
    //Calculate the first time after use GetDeltaTime()
    Timer.Calculate();

    //_swerve.OverrideRotationAxis(_directionHoldPID.Evaluate(Mathf.DeltaAngle(new Polar(1, _swerve.GetHeading()).vector(), new Polar(1, 0.33).vector()), Timer.GetDeltaTime()));

    //Execute Needed Component
    _PneumaticSystem.CheckPressure();
    _thrower.DoThrower();
    _swerve.DoSwerve();
    //_ColorSensor.GetColor();
  } 
}