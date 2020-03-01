package frc.robot.Component;
import frc.robot.Component.Data.SolenoidValve;
import frc.robot.Interface.System;

public class Leds implements System {
    private SolenoidValve green;
    private SolenoidValve blue;
    private SolenoidValve red;

    private String _color = "";
    private Integer _priority = 0;

    public Leds(int greenChannel, int blueChannel, int redChannel) {
        green = SolenoidValve.CreateSingle(greenChannel, 1);
        blue = SolenoidValve.CreateSingle(blueChannel, 1);
        red = SolenoidValve.CreateSingle(redChannel, 1);
    }

    public void Init() {

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

            case "off":
            default:
                green.SetState(false);
                blue.SetState(false);
                red.SetState(false);
            break;
        }
    }
}