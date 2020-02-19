package frc.robot.Component.Data;

import java.util.HashMap;

/**
 * A class that contain the initialization of the input
 */
public class InputManager 
{
    private static String[] _allButton;

    private static HashMap<String, Boolean> _lastState = new HashMap<String, Boolean>();
    private static HashMap<String, Boolean> _currentState = new HashMap<String, Boolean>();

    /**
     * Initialize the input of the project
     */
    public static void Init()
    {
        //Swerve
        Input.CreateAxis("Horizontal", 0, 0, false);
        Input.CreateAxis("Vertical", 0, 1, false);
        Input.CreateAxis("Rotation", 0, 3, false);

        Input.SetAxisNegative("Rotation", 0, 2, false);

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

        //Climber
        Input.CreateButton("ToggleClimber", 0, 12);

        _allButton = Input.GetAllButton();
        for (String key : _allButton)
        {
            _lastState.put(key, false);
            _currentState.put(key, false);
        }
    }

    /**
     * Compute the current input manager
     */
    public static void DoInputManager()
    {
        for (String key : _allButton)
        {
            _lastState.put(key, _currentState.get(key));
            _currentState.put(key, Input.GetButton(key));
        }
    }

    /**
     * Get if the button is currently being pressed
     * @param ButtonName The name of the button to check
     * @return If the button nis currently being pressed
     */
    public static boolean GetButton(String ButtonName)
    {
        return _currentState.get(ButtonName);
    }
    /**
     * Get if the button just got pressed down
     * @param ButtonName The name of the button to check
     * @return If the button just got pressed down
     */
    public static boolean GetButtonDown(String ButtonName)
    {
        return !_lastState.get(ButtonName) && _currentState.get(ButtonName);
    }
    /**
     * Get if the button just got release
     * @param ButtonName The name of the button to check
     * @return If the button just got release
     */
    public static boolean GetButtonUp(String ButtonName)
    {
        return _lastState.get(ButtonName) && !_currentState.get(ButtonName);
    }
}
