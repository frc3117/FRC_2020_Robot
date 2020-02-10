package frc.robot.Component;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Component.Data.*;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Math.PID;
import frc.robot.Math.Mathf;

public class BallThrower 
{
    public BallThrower(Swerve swerve, int AllignButton, int ShootButton, double IdleRPM, double ShootRPM)
    {
        if(!Input.ContainButton("Allign"))
        {
            Input.CreateButton("Align", 0, AllignButton);
        }
        if(!Input.ContainButton("Shoot"))
        {
            Input.CreateButton("Shoot", 0, ShootButton);
        }

        _swerve = swerve;
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

    private Swerve  _swerve;
    private MotorController _feederController = new MotorController(MotorControllerType.TalonSRX, 2, false);
    private MotorController _elevationController;
    private MotorController[] _inertiaWheelControler;
    private Encoder _inertiaWheelEncoder;
    private LimeLight _limeLight = new LimeLight();
    private PID _directionPID = new PID(0.06, 0.02, 0.000);
    private PID _elevationPID = new PID(0.035, 0.05, 0);
    private PID _inertiaWheelPID = new PID(0, 0, 0, "Speed");

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

        if(_isAllign || true)
        {
            /*LimeLightData currentData = _limeLight.GetCurrent();
            
            //Only try to align if there is a target in the line of sight
            if(currentData.IsTarget())
            {
                double rotationAxis = _directionPID.Evaluate(currentData.GetAngleX(), Timer.GetDeltaTime());
                double elevationAxis = _elevationPID.Evaluate(currentData.GetAngleY(), Timer.GetDeltaTime()) * -1;

              _swerve.OverrideRotationAxis(Mathf.Clamp(rotationAxis, -1, 1));
              _elevationController.Set(Mathf.Clamp(elevationAxis, -1, 1));
            }

            if((!_isAutoShoot || currentData.GetAngleX() + currentData.GetAngleY() <= _errorTolerency))
            {
                _isReady = true;
            }*/

            if(_isAutoShoot || Input.GetButton("Shoot"))
            {
                double RPM = ((_inertiaWheelEncoder.getRate() / 2048) * 60);
                SmartDashboard.putNumber("Velocity", RPM);

                double val = Mathf.Clamp(_inertiaWheelPID.Evaluate(_shootRPM - RPM), 0.1, 1);

                for (MotorController motorController : _inertiaWheelControler) 
                {
                    motorController.Set(val);
                }

                if(RPM >= _shootRPM - 200)//if(_isReady)
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
