package frc.robot.Component.Data;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * The universal class for the solenoid valve
 */
public class SolenoidValve 
{
    private SolenoidValve(int ChannelA)
    {
        _type = SolenoidType.Single;
        _singleSolenoid = new Solenoid(ChannelA);
    }
    private SolenoidValve(int ChannelA, int ChannelB)
    {
        _type = SolenoidType.Double;
        _doubleSolenoid = new DoubleSolenoid(ChannelA, ChannelB);
    }

    private enum SolenoidType
    {
        Single,
        Double
    }

    private SolenoidType _type;

    private Solenoid _singleSolenoid;
    private DoubleSolenoid _doubleSolenoid;

    /**
     * Create a single action solenoid valve
     * @param ChannelA The channel of the solenoid valve
     * @return The single action solenoid valve
     */
    public static SolenoidValve CreateSingle(int ChannelA)
    {
        return new SolenoidValve(ChannelA);
    }
    /**
     * Create a double action solenoid valve
     * @param ChannelA The forward channel of the solenoid valve
     * @param ChannelB The reverse channel of the solenoid valve
     * @return The double action solenoid valve
     */
    public static SolenoidValve CreateDouble(int ChannelA, int ChannelB)
    {
        return new SolenoidValve(ChannelA, ChannelB);
    }

    /**
     * Set the current state of the solenoid valve
     * @param State The state to set the solenoid valve to
     */
    public void SetState(boolean State)
    {
        switch(_type)
        {
            case Single:
            _singleSolenoid.set(State);
            break;

            case Double:
            _doubleSolenoid.set(State ? Value.kForward : Value.kReverse);
            break;
        }
    }
    /**
     * Get the current state of the solenoid valve
     * @return The current state of the current valve
     */
    public boolean GetState()
    {
        switch(_type)
        {
            case Single:
            return _singleSolenoid.get();

            case Double:
            return _doubleSolenoid.get() == Value.kForward;
        }

        return false;
    }
}
