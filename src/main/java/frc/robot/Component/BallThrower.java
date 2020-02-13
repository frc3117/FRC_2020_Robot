package frc.robot.Component;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Component.Data.*;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Math.PID;
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

    private Servo _throwerServo;

    private double _idleRPM;
    private double _shootRPM;

    private boolean _isAutoShoot = false;

    private boolean _isAllign = false;
    private boolean _isReady = false;

    private boolean _isLimitSwitch = false;

    private boolean _alignButtonLastState = false;

    private double _errorTolerency = 5;

    private double angle;

    private DigitalInput _limitSwitch1 = new DigitalInput(4);
    private DigitalInput _limitSwitch2 = new DigitalInput(5);

    public void Init()
    {
        _isAutoShoot = false;
        _isAllign = false;
        _isReady = false;

        _limeLight.SetRecognitionMode();
    }

    public double GetDistance()
    {
        LimeLightData current = _limeLight.GetCurrent();
        double camAngle = (CamAngle + current.GetAngleY()) * Mathf.DEG_2_RAD;

        return (1 / (Math.tan(camAngle))) * (TargetHeight - CamHeight);
    }

    public void DoThrower()
    {
        if(!_isLimitSwitch)
        {
            if(_limitSwitch1.get() || _limitSwitch2.get())
            {
                _isLimitSwitch = true;
                _throwerServo.setPosition(0.5);
            }
            else
            {
                _throwerServo.setPosition(0.45);
            }
        }

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

        if(_isAllign)
        {
            LimeLightData current = _limeLight.GetCurrent();

            if(current.IsTarget())
            {
                double distance = GetDistance(); //Multiple way of doing it
                double throwerTarget = 0; //Need to find the equatio

                //Allign to the target
                if(Math.signum(current.GetAngleY()) == 1)
                {
                    //_camServo.setAngle(_camServo.getAngle() - 0.1);
                }
                else
                {
                    //_camServo.setAngle(_camServo.getAngle() + 0.1);
                }
                Robot.SwerveDrive.OverrideRotationAxis(_directionPID.Evaluate(current.GetAngleX()));
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

                if(RPM >= _shootRPM - 200)
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
