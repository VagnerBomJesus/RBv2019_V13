import com.ridgesoft.robotics.AnalogInput;

public class NewWayFlameSensor implements FlameSensor {

	private AnalogInput[] mSensors = new AnalogInput[5];
	private int mIsPresentLimit;
	private int mIsNearLimit;  //Not used.

	private int mFlameValue;
	private int mFlameDirection;

	public NewWayFlameSensor(AnalogInput sensors[], int isPresentLimit, int isNearLimit) {
		mSensors = sensors;
		mIsPresentLimit = isPresentLimit;
		mIsNearLimit = isNearLimit;

		mFlameValue = 0;
		mFlameDirection = -1;
	}

	public int scan() {
		// Calculates the maximum value of the sensor and the correspondent
		// direction of the flame.
		mFlameValue = 0;
		int v = 0;
		for (int i = 0; i < 5; i++) {
			v = mSensors[i].sample();
			if (v > mFlameValue) {
				mFlameValue = v;
				mFlameDirection = i + 1;
			}
		}

		if (mFlameValue < mIsPresentLimit)
			mFlameDirection = -1; // Flame is not present so returns -1.
		
		//Idea for the mIsNearLimit
		//else if (mFlameValue > mIsNearLimit && mFlameDirection == 3)
		//mFlameDirection = 0; // Flame is present and near so return 0;

		return mFlameDirection;
	}

	public int getDirection() {
		// Get the last known direction of the flame.
		return mFlameDirection;
	}

	public int getValue() {
		// Get the maximum analog value of the sensors.
		return mFlameValue;
	}

	public int getSensor(int i) {
		//Get analog value of sensor i.
		return mSensors[i].sample();
	}
}
