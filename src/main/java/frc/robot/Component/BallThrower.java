package frc.robot.Component;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Component.Data.*;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Interface.System;
import frc.robot.Math.PID;
import frc.robot.Math.Curve;
import frc.robot.Math.Mathf;

public class BallThrower implements System
{
    public BallThrower(double IdleRPM, double ShootRPM)
    {
        _inertiaWheelControler = new MotorController[]
        {
            new MotorController(MotorControllerType.TalonSRX, 3, false),
            new MotorController(MotorControllerType.TalonSRX, 5, false),
            new MotorController(MotorControllerType.TalonSRX, 10, false),
        };
        _inertiaWheelEncoder = new Encoder(9, 8);

        _throwerServo = new Servo(1);

        _limeLight.SetDriveMode();

        SmartDashboard.putNumber("RPM_Offset", 200);

        _idleRPM = IdleRPM;
        _shootRPM = ShootRPM;
    }

    public static final double CamHeight = .67056;
    public static final double CamAngle = 26;
    public static final double TargetHeight = 2.254;

    private MotorController _conveyorBelt = new MotorController(MotorControllerType.TalonFX, 11, false);
    private MotorController _feederController = new MotorController(MotorControllerType.TalonSRX, 2, false);
    private MotorController[] _inertiaWheelControler;
    private Encoder _inertiaWheelEncoder;
    private LimeLight _limeLight = new LimeLight();
    private PID _directionPID = new PID(0.03, 0.01, 0.000);
    private PID _inertiaWheelPID = new PID(0.007, 0, 0, "Speed");

    private Joystick joystick = new Joystick(0);

    private Curve _throwerDistanceCurve = new Curve(
        new Vector2d(-20.15, 50),
        new Vector2d(-18.5, 60),
        new Vector2d(-17.72, 70),
        new Vector2d(-16.32, 80),
        new Vector2d(-14.6, 90),
        new Vector2d(-13.6, 60),
        new Vector2d(-12, 20),
        new Vector2d(-7.6, 0)      
    );

    private Curve _lateralMovementCompensationCurve = new Curve(

    );

    private Servo _throwerServo;

    private double _idleRPM;
    private double _shootRPM;

    private boolean _isAutoShoot = false;

    private boolean _isAllign = false;
    private boolean _isAllignOverriden = false;
    private boolean _isReady = false;

    private boolean _climbMode = false;;

    private double _errorTolerency = 5;

    public void Init()
    {
        _isAutoShoot = false;
        _isAllign = false;
        _isReady = false;

        _climbMode = false;

        _feederController.Set(0);
        _conveyorBelt.Set(0);
    }

    public double GetDistance()
    {
        LimeLightData current = _limeLight.GetCurrent();
        double Angle = (CamAngle + current.GetAngleY()) * Mathf.DEG_2_RAD;

        return (1 / Math.tan(Angle)) * (TargetHeight - CamHeight);
    }
    
    public void StartOverrideAlign()
    {
        _isAllignOverriden = true;
        _limeLight.SetRecognitionMode();
    }
    public void StopOverrideAlign()
    {
        _isAllignOverriden = false;
        _limeLight.SetDriveMode();
    }

    public void SetClimbMode(boolean State)
    {
        _climbMode = State;
    }

    int frame = 0;
    public void DoSystem()
    {
        if(_climbMode)
        {
            for(MotorController motorController : _inertiaWheelControler)
            {
                //Controll the Climber
            }

            _feederController.Set(0);
            _conveyorBelt.Set(0);

            return;
        }
        
        if(!_isAllignOverriden)
        {
            if(InputManager.GetButtonDown("Align"))
            {
                _isAllign = !_isAllign;

                if(_isAllign)
                {
                    _limeLight.SetRecognitionMode();
                }
                else
                {
                    _limeLight.SetDriveMode();
                }
            }
        }

        if(_isAllign || _isAllignOverriden)
        {
            joystick.setRumble(RumbleType.kLeftRumble, 0.5);
            joystick.setRumble(RumbleType.kRightRumble, 0.5);

            Robot.SwerveDrive.OverrideShift(1);

            SmartDashboard.putBoolean("IsAlign", true);

            LimeLightData current = _limeLight.GetCurrent();
            if(current.IsTarget())
            {
                double throwerTarget = _throwerDistanceCurve.Evaluate(current.GetAngleY());

                SmartDashboard.putNumber("ThrowerAngle", throwerTarget);

                _throwerServo.setAngle(throwerTarget);
                Robot.SwerveDrive.OverrideRotationAxis(_directionPID.Evaluate(current.GetAngleX() - 1.5 /*+ _lateralMovementCompensationCurve.Evaluate(Robot.SwerveDrive.GetInstantHorizontalAxis())*/));
            }

            if(_isAutoShoot || Input.GetButton("Shoot"))
            {
                double RPM = ((_inertiaWheelEncoder.getRate() / 2048) * 60) * -1;
                double RPM_Offset = SmartDashboard.getNumber("RPM_Offset", 0);

                SmartDashboard.putNumber("Velocity", RPM);

                double val = Mathf.Clamp(_inertiaWheelPID.Evaluate(_shootRPM - RPM), 0.1, 1);

                for (MotorController motorController : _inertiaWheelControler) 
                {
                    motorController.Set(val);
                }

                _isReady = true;
                _isReady &= Math.abs(current.GetAngleX()) <= _errorTolerency;
                _isReady &= RPM >= _shootRPM - RPM_Offset;

                if(_isReady)
                {
                    //Feed Ball
                    _feederController.Set(1);
                    _conveyorBelt.Set(0.3);
                }
                else
                {
                    _feederController.Set(0);
                    _conveyorBelt.Set(0);
                }
            }
            else
            {
                double RPM = ((_inertiaWheelEncoder.getRate() / 2048) * 60) * -1;              

                SmartDashboard.putNumber("Velocity", RPM);

                double val = Mathf.Clamp(_inertiaWheelPID.Evaluate(_idleRPM - RPM), 0.1, 1);
                
                for (MotorController motorController : _inertiaWheelControler) 
                {
                    motorController.Set(val);
                }

                _feederController.Set(0);
                _conveyorBelt.Set(0);
            }
        }
        else
        {
            joystick.setRumble(RumbleType.kLeftRumble, 0);
            joystick.setRumble(RumbleType.kRightRumble, 0);

            SmartDashboard.putBoolean("IsAlign", false);

            _directionPID.Reset();
            _conveyorBelt.Set(0);
            _feederController.Set(0);

            double RPM = (_inertiaWheelEncoder.getRate() / 2048) * 60 * -1;

            double val = Mathf.Clamp(_inertiaWheelPID.Evaluate(_idleRPM - RPM), 0.1, 1);

            SmartDashboard.putNumber("Velocity", RPM);

            for (MotorController motorController : _inertiaWheelControler) 
            {
                motorController.Set(val);
            }
        }
    }

    public void SetErrorTolerency(double tolerency)
    {
        _errorTolerency = tolerency;
    }
    public void SetAutoShoot(boolean state)
    {
        _isAutoShoot = state;
    }
}
