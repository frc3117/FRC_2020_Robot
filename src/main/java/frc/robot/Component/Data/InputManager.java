package frc.robot.Component.Data;

/**
 * A class that contain the initialization of the input
 */
public class InputManager 
{
    /**
     * Initialize the input of the project
     */
    public static void Init()
    {
        //Swerve
        Input.CreateAxis("Horizontal", 0, 0, false);
        Input.CreateAxis("Vertical", 0, 1, false);
        Input.CreateAxis("Rotation", 0, 3, false);

        Input.SetAxisNegative("Rotation", 0, 1, false);

        Input.SetAxisDeadzone("Horizontal", 0.2);
        Input.SetAxisDeadzone("Vertical", 0.2);

        Input.CreateButton("GearShift", 0, 3);

        //Intake
        Input.CreateButton("ToggleIntake", 0, 4);
        Input.CreateButton("StartFeeder", 0, 6);
        Input.CreateButton("ReverseFeeder", 0, 5);

        //Thrower
        Input.CreateButton("Align", 0, 1);
        Input.CreateButton("Shoot", 0, 2);
    }
}
