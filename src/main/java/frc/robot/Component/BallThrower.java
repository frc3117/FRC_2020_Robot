package frc.robot.Component;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Component.Data.Input;
import frc.robot.Component.Data.InputManager;
import frc.robot.Component.Data.LimeLightData;
import frc.robot.Component.Data.MotorController;
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
            new MotorController(MotorControllerType.SparkMax, 15, true),
            new MotorController(MotorControllerType.SparkMax, 17, true),
        };

        _inertiaWheelControler[0].SetInverted(true);
        
        _inertiaWheelEncoder = new Encoder(9, 8);

        _throwerServo = new Servo(1);

        LimeLight.SetDriveMode();

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
    private PID _directionPID = new PID(0.03, 0.06, 0.000);
    private PID _inertiaWheelPID = new PID(0.007, 0, 0, "Speed");

    private Joystick joystick = new Joystick(0);

    private Curve _throwerDistanceCurve = new Curve(
        new Vector2d(-16, 144),
        new Vector2d(-14, 110),
        new Vector2d(-12, 89),
        new Vector2d(-10, 81),
        new Vector2d(-8, 85),
        new Vector2d(-6, 102),
        new Vector2d(-4, 132),
        new Vector2d(-2, 175),
        new Vector2d(-1, 180),
        new Vector2d(0, 180)
    );

    private Servo _throwerServo;

    private double _idleRPM;
    private double _shootRPM;

    private boolean _isAutoShoot = false;

    private boolean _isAllign = false;
    private boolean _isAllignOverriden = false;
    private boolean _isReady = false;

    private double _errorTolerency = 5;

    public void Init()
    {
        _isAutoShoot = false;
        _isAllign = false;
        _isReady = false;

        _feederController.Set(0);
        _conveyorBelt.Set(0);
    }

    public double GetDistance()
    {
        LimeLightData current = LimeLight.GetCurrent();
        double Angle = (CamAngle + current.GetAngleY()) * Mathf.DEG_2_RAD;

        return (1 / Math.tan(Angle)) * (TargetHeight - CamHeight);
    }
    
    public void StartOverrideAlign()
    {
        _isAllignOverriden = true;
        LimeLight.SetRecognitionMode();
    }
    public void StopOverrideAlign()
    {
        _isAllignOverriden = false;
        LimeLight.SetDriveMode();
    }

    int frame = 0;
    public void DoSystem()
    {
        if(!_isAllignOverriden)
        {
            if(InputManager.GetButtonDown("Align"))
            {
                _isAllign = !_isAllign;

                if(_isAllign)
                {
                    LimeLight.SetRecognitionMode();
                }
                else
                {
                    LimeLight.SetDriveMode();
                }
            }
        }

        if(_isAllign || _isAllignOverriden)
        {
            joystick.setRumble(RumbleType.kLeftRumble, 0.5);
            joystick.setRumble(RumbleType.kRightRumble, 0.5);

            Robot.Leds.SetColor("lightpurple", 0, 0);

            Robot.SwerveDrive.OverrideShift(1);

            SmartDashboard.putBoolean("IsAlign", true);
            //SmartDashboard.putNumber("ThrowerAngle", 0);

            LimeLightData current = LimeLight.GetCurrent();

            if(current.IsTarget() && !(Input.GetButton("Shoot") || _isAutoShoot))
            {
                double throwerTarget = _throwerDistanceCurve.Evaluate(current.GetAngleY());

                if (Math.abs(current.GetAngleX()) <= _errorTolerency) {
                    Robot.Leds.SetColor("blue", 0, 0);
                } else {
                    Robot.Leds.SetColor("lime", 0, 0);
                }

                SmartDashboard.putNumber("Angle", current.GetAngleY());
                SmartDashboard.putNumber("ThrowerAngle", throwerTarget);

                _throwerServo.setAngle(throwerTarget);
                Robot.SwerveDrive.OverrideRotationAxis(_directionPID.Evaluate(current.GetAngleX()));
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

                _isReady = current.IsTarget();
                _isReady &= Math.abs(current.GetAngleX()) <= _errorTolerency;
                _isReady &= RPM >= _shootRPM - RPM_Offset;

                if(_isReady)
                {
                    Robot.Leds.SetColor("green", 0, 0);
                    //Feed Ball
                    _feederController.Set(1);
                    _conveyorBelt.Set(0.3);
                }
                else
                {
                    if (RPM < _shootRPM - RPM_Offset) {
                        Robot.Leds.SetColor("red", 0, 0);
                    }

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

            Robot.Leds.SetColor("white", 0, 0);

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
