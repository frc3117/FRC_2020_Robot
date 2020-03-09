package frc.robot.Component;
import java.util.Random;

import frc.robot.Component.Data.SolenoidValve;
import frc.robot.Interface.Component;

public class Leds implements Component {
    private SolenoidValve green;
    private SolenoidValve blue;
    private SolenoidValve red;

    private String _color = "";
    private Integer _priority = 0;

    private CycleKey[] _cycle;
    private int _currentCycleIndex = 0;
    private boolean _isCycle = false;

    public Leds(int greenChannel, int blueChannel, int redChannel) {
        green = SolenoidValve.CreateSingle(greenChannel, 1);
        blue = SolenoidValve.CreateSingle(blueChannel, 1);
        red = SolenoidValve.CreateSingle(redChannel, 1);
    }

    public void Init() {
        _color = "off";
        _priority = 0;
    }

    public void SetColorCycle(String Cycle)
    {

    }

    public void SetColor(String color, Integer priority, Integer newPriority) {
        if (priority >= _priority) {
            _priority = newPriority;
            _color = color;
        }
    }

    public void DoSystem() {    
        switch (_color) {
            case "green":
                green.SetState(true);
                blue.SetState(false);
                red.SetState(false);
            break;

            case "blue":
                green.SetState(false);
                blue.SetState(true);
                red.SetState(false);
            break;

            case "red":
                green.SetState(false);
                blue.SetState(false);
                red.SetState(true);
            break;

            case "lightblue":
                green.SetState(true);
                blue.SetState(true);
                red.SetState(false);
            break;

            case "lime":
                green.SetState(true);
                blue.SetState(false);
                red.SetState(true);
            break;

            case "lightpurple":
                green.SetState(false);
                blue.SetState(true);
                red.SetState(true);
            break;

            case "white":
                green.SetState(true);
                blue.SetState(true);
                red.SetState(true);
            break;

            case "random":
                green.SetState(new Random().nextInt(2) == 1);
                blue.SetState(new Random().nextInt(2) == 1);
                green.SetState(new Random().nextInt(2) == 1);
            break;

            case "off":
            default:
                green.SetState(false);
                blue.SetState(false);
                red.SetState(false);
            break;
        }
    }

    private class CycleKey
    {
        public String Color;
        public double Time;
    }
}