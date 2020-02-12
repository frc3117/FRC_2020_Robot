package frc.robot.Component;

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

        _elevationController = new MotorController(MotorController.MotorControllerType.TalonSRX, 6, false);

        _inertiaWheelControler = new MotorController[]
        {
            new MotorController(MotorControllerType.TalonSRX, 3, false),
            new MotorController(MotorControllerType.TalonSRX, 5, false),
            new MotorController(MotorControllerType.TalonSRX, 10, false),
        };
        _inertiaWheelEncoder = new Encoder(9, 8);

        _limeLight.SetDriveMode();

        _idleRPM = IdleRPM;
        _shootRPM = ShootRPM;
    }

    public static final double CamHeight = 0;
    public static final double TargetHeight = 0;

    private MotorController _feederController = new MotorController(MotorControllerType.TalonSRX, 2, false);
    private MotorController _elevationController;
    private MotorController[] _inertiaWheelControler;
    private Encoder _inertiaWheelEncoder;
    private LimeLight _limeLight = new LimeLight();
    private PID _directionPID = new PID(0.06, 0.02, 0.000);
    private PID _elevationPID = new PID(0.035, 0.05, 0);
    private PID _inertiaWheelPID = new PID(0.007, 0, 0, "Speed");

    private Servo _camServo;
    private Servo _throwerServo;

    private double _idleRPM;
    private double _shootRPM;

    private boolean _isAutoShoot = false;

    private boolean _isAllign = false;
    private boolean _isReady = false;

    private boolean _alignButtonLastState = false;

    private double _errorTolerency = 5;

    public void Init()
    {
        _isAutoShoot = false;
        _isAllign = false;
        _isReady = false;
    }

    public void DoThrower()
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

        if(_isAllign)
        {
            double camAngle = _camServo.getAngle() * Mathf.DEG_2_RAD;
            double distance = (1 / (Math.tan(camAngle))) * (TargetHeight - CamHeight); //Multiple way of doing it

            double throwerTarget = 0; //Need to find the equation

            if(_isAutoShoot || Input.GetButton("Shoot"))
            {
                Robot.Intake.OverrideConveyorBelt();

                double RPM = ((_inertiaWheelEncoder.getRate() / 2048) * 60);
                SmartDashboard.putNumber("Velocity", RPM);

                double val = Mathf.Clamp(_inertiaWheelPID.Evaluate(_shootRPM - RPM), 0.1, 1);

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
                double RPM = ((_inertiaWheelEncoder.getRate() / 2048) * 60);
                SmartDashboard.putNumber("Velocity", RPM);                

                double val = Mathf.Clamp(_inertiaWheelPID.Evaluate(_idleRPM - RPM), 0, 1);

                for (MotorController motorController : _inertiaWheelControler) 
                {
                    motorController.Set(val);
                }

                _feederController.Set(0);
            }
        }
        else
        {
            _elevationController.Set(0);
            _directionPID.Reset();
            _elevationPID.Reset();

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
