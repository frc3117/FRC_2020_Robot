package frc.robot.Component;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Component.Data.*;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Math.PID;
import frc.robot.Math.Curve;
import frc.robot.Math.Mathf;

public class BallThrower 
{
    public BallThrower(int AllignButton, int ShootButton, double IdleRPM, double ShootRPM)
    {
        if(!Input.ContainButton("Allign"))
        {
            Input.CreateButton("Align", 0, AllignButton);
        }
        if(!Input.ContainButton("Shoot"))
        {
            Input.CreateButton("Shoot", 0, ShootButton);
        }

        _inertiaWheelControler = new MotorController[]
        {
            new MotorController(MotorControllerType.TalonSRX, 3, false),
            new MotorController(MotorControllerType.TalonSRX, 5, false),
            new MotorController(MotorControllerType.TalonSRX, 10, false),
        };
        _inertiaWheelEncoder = new Encoder(9, 8);

        _throwerServo = new Servo(1);

        _limeLight.SetDriveMode();

        _idleRPM = IdleRPM;
        _shootRPM = ShootRPM;
    }

    public static final double CamHeight = 0;
    public static final double CamAngle = 0;
    public static final double TargetHeight = 0;

    private MotorController _feederController = new MotorController(MotorControllerType.TalonSRX, 2, false);
    private MotorController[] _inertiaWheelControler;
    private Encoder _inertiaWheelEncoder;
    private LimeLight _limeLight = new LimeLight();
    private PID _directionPID = new PID(0.06, 0.02, 0.000);
    private PID _inertiaWheelPID = new PID(0.007, 0, 0, "Speed");

    private Curve _throwerDistanceCurve = new Curve();

    private Servo _throwerServo; // 1=Up 0.5=Stop 0=Down

    private double _idleRPM;
    private double _shootRPM;

    private boolean _isAutoShoot = false;

    private boolean _isAllign = false;
    private boolean _isAllignOverriden = false;
    private boolean _isReady = false;

    private boolean _isLimitSwitch = false;

    private boolean _alignButtonLastState = false;

    private double _errorTolerency = 5;

    private DigitalInput _limitSwitch1 = new DigitalInput(5);
    private DigitalInput _limitSwitch2 = new DigitalInput(6);

    public void Init()
    {
        _isAutoShoot = false;
        _isAllign = false;
        _isReady = false;

        _isLimitSwitch = false;

        //_limeLight.SetRecognitionMode();
    }

    public double GetDistance()
    {
        LimeLightData current = _limeLight.GetCurrent();
        double camAngle = (CamAngle + current.GetAngleY()) * Mathf.DEG_2_RAD;

        return (1 / (Math.tan(camAngle))) * (TargetHeight - CamHeight);
    }
    
    public void OverrideAlign()
    {
        _isAllignOverriden = true;
    }

    public void DoThrower()
    {
        if(!_isLimitSwitch)
        {
            if(!_limitSwitch1.get())
            {
                _isLimitSwitch = true;
                _throwerServo.set(0.5);
            }
            else
            {
                _throwerServo.set(0);
            }

            return;
        }

        if(!_isAllignOverriden)
        {
            boolean allignButton = Input.GetButton("Align");
            if(allignButton && !_alignButtonLastState)
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
            _alignButtonLastState = allignButton;
        }
        else
        {
            _alignButtonLastState = false;
        }

        if(_isAllign || _isAllignOverriden)
        {
            LimeLightData current = _limeLight.GetCurrent();

            if(current.IsTarget())
            {
                double distance = GetDistance(); //Multiple way of doing it
                double throwerTarget = current.GetAngleY() - _throwerDistanceCurve.Evaluate(distance); //Need to find the equatio

                //Allign to the target
                if(Math.abs(throwerTarget) <= 1)
                {
                    double min;
                    double max;

                    if(!_limitSwitch1.get())
                    {
                        max = 0.5;
                        min = 0;
                    }
                    else if (!_limitSwitch2.get())
                    {
                        max = 1;
                        min = 0.5;
                    }
                    else
                    {
                        max = 1;
                        min = 0;
                    }

                    _throwerServo.set(Mathf.Clamp(Math.signum(throwerTarget), min, max));
                    _isReady = false;
                }
                else
                {
                    _throwerServo.set(0.5);
                    _isReady = true;
                }
                Robot.SwerveDrive.OverrideRotationAxis(_directionPID.Evaluate(current.GetAngleX()));
            }
            else
            {
                _isReady = false;
                //Try Sweeping for detection?
            }

            if(_isAutoShoot || Input.GetButton("Shoot"))
            {
                double RPM = ((_inertiaWheelEncoder.getRate() / 2048) * 60) * -1;
                SmartDashboard.putNumber("Velocity", RPM);

                double val = Mathf.Clamp(_inertiaWheelPID.Evaluate(_shootRPM - RPM), -1, -0.1);

                for (MotorController motorController : _inertiaWheelControler) 
                {
                    motorController.Set(val);
                }

                if(RPM >= _shootRPM - 200 && _isReady)
                {
                    //Feed Ball
                    _feederController.Set(1);
                }
                else
                {
                    _feederController.Set(0);
                }
            }
            else
            {
                double RPM = ((_inertiaWheelEncoder.getRate() / 2048) * 60) * -1;
                SmartDashboard.putNumber("Velocity", RPM);                

                double val = Mathf.Clamp(_inertiaWheelPID.Evaluate(_idleRPM - RPM), -1, -0.1);

                for (MotorController motorController : _inertiaWheelControler) 
                {
                    motorController.Set(val);
                }

                _feederController.Set(0);
            }
        }
        else
        {
            _directionPID.Reset();

            double val = Mathf.Clamp(_inertiaWheelPID.Evaluate(_idleRPM - (_inertiaWheelEncoder.getRate() / 34.1333333333)), -1, 0);

            for (MotorController motorController : _inertiaWheelControler) 
            {
                motorController.Set(val);
            }
        }

        _isAllignOverriden = true;
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
