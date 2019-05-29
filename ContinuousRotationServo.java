import com.ridgesoft.robotics.Motor;
import com.ridgesoft.robotics.Servo;

public class ContinuousRotationServo implements Motor {
	private Servo mServo;
	private boolean mReverse;
	private int mRange;

	public ContinuousRotationServo(Servo servo, boolean reverse, int range) {
		mServo = servo;     // Object to represent the servo motor.
		mReverse = reverse; // Flag to that indicates if the value of the power should by inverted.
		mRange = range;     // Value to define the effective range above and below the neutral value 0f 50.
	}

	public void brake() {
		mServo.setPosition(50);
	}

	public void setPower(int power) {
		// If mReverse is true, invert the sense of direction for this servo motor.
		if (mReverse)
			power = -power; 

		// Turn servo off when the power is set to zero and limit the range of the power variable. 
		if (power == 0) {
			mServo.off();
			return;
		} else if (power > Motor.MAX_FORWARD)
			power = Motor.MAX_FORWARD; // 16
		else if (power < Motor.MAX_REVERSE)
			power = Motor.MAX_REVERSE; // -16

		// Convert the value of power into a value of range for the setPosition().
		// Motor.MAX_FORWARD -> mRange
		// power -> X
		mServo.setPosition((power * mRange) / Motor.MAX_FORWARD + 50);
	}

	public void stop() {
		mServo.off();
	}
}
