package frc.robot.Component;

import frc.robot.Robot;
import frc.robot.Component.Data.ClimbingLED;
import frc.robot.Component.Data.InputManager;
import frc.robot.Component.Data.MotorController;
import frc.robot.Component.Data.MotorController.MotorControllerType;
import frc.robot.Interface.Component;

public class Climber implements Component
{
    public Climber(int MotorChannel)
    {
        _climberMotor = new MotorController(MotorControllerType.SparkMax, MotorChannel, true);
    }

    private MotorController  _climberMotor;
    private ClimbingLED _ledAnimation = new ClimbingLED(3, 1);

    public void Init()
    {
        _ledAnimation.Init();
    }

    public void DoSystem()
    {
        
        if(InputManager.GetButtonDown("Temp"))
        {
            _ledAnimation.StartAnimation();
        }
        _ledAnimation.DoSystem();

        if(InputManager.GetButton("ClimberUp"))
        {
            _climberMotor.Set(-0.35);
            Robot.Intake.OpenIntake();
        }
        else if (InputManager.GetButton("ClimberDown"))
        {
            _climberMotor.Set(0.75);
            Robot.Intake.OpenIntake();
        }
        else
        {
            _climberMotor.Set(0);
        }
    }
}
