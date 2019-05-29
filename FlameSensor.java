
public interface FlameSensor {
	public int scan();            // Performs a scan and returns the direction of the flame.
	public int getDirection();    // Returns the last known direction.
	public int getValue();        // Returns the maximum raw value (radiation value).
}
