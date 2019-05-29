import com.ridgesoft.intellibrain.IntelliBrain;
import com.ridgesoft.intellibrain.IntelliBrainDigitalIO;
import com.ridgesoft.io.Display;
import com.ridgesoft.robotics.AnalogInput;
import com.ridgesoft.robotics.Motor;
import com.ridgesoft.robotics.RangeFinder;
import com.ridgesoft.robotics.sensors.ParallaxPing;

public class RBv2019 {

	// ================================================================================
	// Constants
	// ================================================================================

	// Possible states.
	private static final int WAIT = 0; // Wait for the Start button
	private static final int NAVIGATE_RIGHT = 1; // Navigate by following the right wall.
	private static final int CENTER = 2; // Center the robot in relation to the candle flame.
	private static final int PUT_OUT = 3; // Put out the candle flame.
	private static final int RETURN = 4;
	private static final int NAVIGATE_LEFT = 5;
	private static final int NAVIGATE_BACK = 6;
	private static final int RETURN_Q3 = 7;
	private static final int RETURN_Q4 = 8;
	
	
	// Possible types of floor tags.
	private static final int LINE_TAG = 0;
	private static final int CIRCLE_TAG = 1;
	private static final int NO_TAG = 2;

	// Version
	private static final String VERSION = "RB2019";

	// Predefined velocities.
	private static final int BASE_POWER = 9; // Base power to move.
	private static final int ROTATE_POWER = 5; // Base power to rotate.

	// Other constants used in the algorithms.
	private static final float ANGLE_TO_TIME_FACTOR = 7.5f; // Factor used by the rotateAngle()

	private static final int MIN_DISTANCE_FRONT = 16; // Minimum distance to front wall.
	private static final int MIN_DISTANCE_RIGHT = 16; // Minimum distance to right wall.
	private static final int MIN_DISTANCE_LEFT = 16;
	
	private static final float GAIN = 0.9f; // Gain of the proportional control.
	private static final int DELTA_LIMITE = 5; // Delta limit of the proportional control.

	private static final int LINE_LIMIT = 100; // Limit to detect the white lines.

	private static final int IS_PRESENT_LIMIT = 300; // Limit to detect the flame.
	private static final int IS_NEAR_LIMITE = 1000; // Limit to consider is near the flame.

	// ================================================================================
	// Objects and other variables.
	// ================================================================================

	private static Motor mLeftMotor;
	private static Motor mRightMotor;

	private static RangeFinder mLeftSonar;
	private static RangeFinder mFrontSonar;
	private static RangeFinder mRightSonar;

	private static Display mLcd;

	private static IntelliBrainDigitalIO mLeftBumper;
	private static IntelliBrainDigitalIO mRightBumper;

	private static AnalogInput mLineSensor;
	private static int mTotalLines;

	private static IntelliBrainDigitalIO mStartButton;

	private static IntelliBrainDigitalIO mStopButton;

	private static AnalogInput[] mAnalogSensors;
	private static FlameSensor mFlameSensor;
	private static boolean mFlame;
	private static boolean mFlameInRoom;
	private static int mRoom;

	private static Motor mFan;

	private static IntelliBrainDigitalIO mFlameLED;

	private static AnalogInput mUVTronSensor;
	
	private static boolean dentroQuarto = true;

	public static void main(String[] args) {

		// ================================================================================
		// Creation of the objects.
		// ================================================================================

		mLeftMotor = new ContinuousRotationServo(IntelliBrain.getServo(1), false, 14);
		mRightMotor = new ContinuousRotationServo(IntelliBrain.getServo(2), true, 14);

		mLeftSonar = new ParallaxPing(IntelliBrain.getDigitalIO(3));
		mFrontSonar = new ParallaxPing(IntelliBrain.getDigitalIO(4));
		mRightSonar = new ParallaxPing(IntelliBrain.getDigitalIO(5));

		mLcd = IntelliBrain.getLcdDisplay();

		mLeftBumper = IntelliBrain.getDigitalIO(1);
		mLeftBumper.setPullUp(true);
		mRightBumper = IntelliBrain.getDigitalIO(10);
		mRightBumper.setPullUp(true);

		mLineSensor = IntelliBrain.getAnalogInput(7);

		mStartButton = IntelliBrain.getDigitalIO(12);
		mStartButton.setPullUp(true);

		mStopButton = IntelliBrain.getDigitalIO(11);
		mStopButton.setPullUp(true);

		mAnalogSensors = new AnalogInput[5];
		mAnalogSensors[0] = IntelliBrain.getAnalogInput(1);
		mAnalogSensors[1] = IntelliBrain.getAnalogInput(2);
		mAnalogSensors[2] = IntelliBrain.getAnalogInput(3);
		mAnalogSensors[3] = IntelliBrain.getAnalogInput(4);
		mAnalogSensors[4] = IntelliBrain.getAnalogInput(5);
		mFlameSensor = new NewWayFlameSensor(mAnalogSensors, IS_PRESENT_LIMIT, IS_NEAR_LIMITE);

		mFan = IntelliBrain.getMotor(2);

		mFlameLED = IntelliBrain.getDigitalIO(13);
		mFlameLED.setDirection(true);

		mUVTronSensor = IntelliBrain.getAnalogInput(6);
	

		// ================================================================================
		// State Machine.
		// ================================================================================
		int state = WAIT;
		while (true) {
			switch (state) {
			case WAIT:
				state = waitState();
				break;
			case NAVIGATE_RIGHT:
				state = navigateRightState();
				break;
			case CENTER:
				state = centerState();
				break;
			case PUT_OUT:
				state = putOutState();
				break;
			case RETURN:
				state = returnState();
				break;
			case NAVIGATE_LEFT:
				state = navigateLeftState();
				break;
			case NAVIGATE_BACK:
				state = navigateBackState();
				break;
			case RETURN_Q3:
				state = rQuarto3();
				break;
			case RETURN_Q4:
				state = rQuarto4();
				break;
			}

			displayState(state);
			state = checkStopButton(state);
		}
	}

	// ================================================================================
	// Methods to implement the states.
	// ================================================================================

	private static int waitState() {
		stop();
		mLcd.print(0, VERSION);
		while (mStartButton.isSet()) {
			// testLineSensor();
			mFlameSensor.scan();
			displayFlameSensorData(1);
		}

		// Initializations
		mTotalLines = 0;
		mFlame = true;
		mFlameInRoom = false;
		mRoom = 0;

		maneuverToCorrectDirection();
		maneuverToExitWhiteCircle();

		//return CENTER; 
		return NAVIGATE_RIGHT;
	}

	private static int navigateRightState() {
		// ===== Action of the state =====
		// Rotate left if wall in front.
		if (getDistance(mFrontSonar) < MIN_DISTANCE_FRONT)
			rotateAngle(90);

		checkBumpers(); // Check if bumpers are colliding with something.
		// countLines();
		// countLines2(); // Count white lines.

		// Proportional control
		// float error = (getDistance(mRightSonar) - MIN_DISTANCE_RIGHT);
		// int delta = (int) (error * GAIN);
		int delta = (int) ((getDistance(mRightSonar) - MIN_DISTANCE_RIGHT) * GAIN);

		// Limit the delta to solve situation where the error is to big, like in certain
		// corners.
		delta = (delta > DELTA_LIMITE ? DELTA_LIMITE : delta);

		// Move the robot proportionally to the error.
		move(BASE_POWER, delta);

		// ===== Transition conditions of the state =====

		// If at start circle and flame still lit, than go back to go to island room.
		// Else go to state WAIT.

		int floorTag = getFloorTag();
		if (floorTag == CIRCLE_TAG) {
			if (mFlame == true)
				maneuverToReturnBack();
			else
				return WAIT;
		} else if (floorTag == LINE_TAG) {
			stop();
			wait(1000);
			mRoom++;
			if (mUVTronSensor.sample() > 0)
				mFlameInRoom = true;
			else
				maneuverToGoToNextRoom();
			if (mRoom == 4)
				mRoom = 3;
		}

		// If mFlameSensor returns a valid direction, go to state CENTER.
		// This test should be performed only if the robot is within a room
		// since there is no flames in the corridors. doing so will prevent
		// the robot from confusing the reflection of the sun with the flame
		// while navigating the corridors.
		if (mFlameInRoom && mFlameSensor.scan() > 0) {
			mFlameLED.set();
			return CENTER;
		}

		return NAVIGATE_RIGHT;
	}

	private static int navigateLeftState() {
	
		if (getDistance(mFrontSonar) < MIN_DISTANCE_FRONT)
			rotateAngle(-90);

		checkBumpers();
		int delta = (int) ((getDistance(mLeftSonar) - MIN_DISTANCE_LEFT) * GAIN);

		delta = (delta > DELTA_LIMITE ? DELTA_LIMITE : delta);

		move(BASE_POWER, -delta);

	
		int floorTag = getFloorTag();
		if(floorTag == CIRCLE_TAG) {
			return WAIT;
		}else if(mTotalLines == 2) {
			stop();
		}else if (floorTag == LINE_TAG && dentroQuarto == false) {
			rotateAngle(90);
		}


		return NAVIGATE_LEFT;
	}
	
	private static int navigateBackState() {
		
		if (getDistance(mFrontSonar) < MIN_DISTANCE_FRONT)
			rotateAngle(90);

		checkBumpers(); 
		
		int delta = (int) ((getDistance(mRightSonar) - MIN_DISTANCE_RIGHT) * GAIN);

		// Limit the delta to solve situation where the error is to big, like in certain
		// corners.
		delta = (delta > DELTA_LIMITE ? DELTA_LIMITE : delta);

		// Move the robot proportionally to the error.
		move(BASE_POWER, delta);
		
		
		int floorTag = getFloorTag();
		
		
		
		if ( floorTag == LINE_TAG) {
			move(BASE_POWER, 0);
			wait(500);
			rotateAngle(90);
			move(BASE_POWER, 0);
			wait(5500);
		}
			
			if (floorTag == CIRCLE_TAG) {
					return WAIT;
			}
			
		
		
		/*if (floorTag == LINE_TAG) {
			rotateAngle(100);
			move(BASE_POWER, 0);
			wait(500);
			
			if (floorTag == CIRCLE_TAG) {
				if (mFlame == true)
					maneuverToReturnBack();
				else
					return WAIT;
			}
			
		}*/
		
		/*do {
			move(BASE_POWER,0);
			}while((floorTag == 1));*/


			/*if (getDistance(mFrontSonar) < MIN_DISTANCE_FRONT)
				rotateAngle(90);

			checkBumpers(); 
		
			int delta = (int) ((getDistance(mRightSonar) - MIN_DISTANCE_RIGHT) * GAIN);

			// Limit the delta to solve situation where the error is to big, like in certain
			// corners.
			delta = (delta > DELTA_LIMITE ? DELTA_LIMITE : delta);

			// Move the robot proportionally to the error.
			move(BASE_POWER, delta);
			
			
		
			rotateAngle(90);
			do {
				move(BASE_POWER,0);
				}while((getDistance(mFrontSonar) > MIN_DISTANCE_FRONT));
		
*/
		// ===== Transition conditions of the state =====

		// If at start circle and flame still lit, than go back to go to island room.
		// Else go to state WAIT.

		
		/*if (floorTag == CIRCLE_TAG) {
			if (mFlame == true)
				maneuverToReturnBack();
			else
				return WAIT;
		} else if (floorTag == LINE_TAG) {
			stop();
			wait(1000);
			mRoom++;
			if (mUVTronSensor.sample() > 0)
				mFlameInRoom = true;
			else
				maneuverToGoToNextRoom();
			if (mRoom == 4)
				mRoom = 3;
		}

		if (mFlameInRoom && mFlameSensor.scan() > 0) {
			mFlameLED.set();
			return CENTER;
		}*/

		return NAVIGATE_BACK;
	}
	
	private static int rQuarto3() {
		
		if (getDistance(mFrontSonar) < MIN_DISTANCE_FRONT)
			rotateAngle(90);

		checkBumpers();
		int delta = (int) ((getDistance(mRightSonar) - MIN_DISTANCE_RIGHT) * GAIN);

		delta = (delta > DELTA_LIMITE ? DELTA_LIMITE : delta);

		move(BASE_POWER, delta);

	
		int floorTag = getFloorTag();
		if(floorTag == CIRCLE_TAG) {
			return WAIT;
		}
		
		return RETURN_Q3;
		
	}
private static int rQuarto4() {
		
		if (getDistance(mFrontSonar) < MIN_DISTANCE_FRONT)
			rotateAngle(90);

		checkBumpers();
		int delta = (int) ((getDistance(mRightSonar) - MIN_DISTANCE_RIGHT) * GAIN);

		delta = (delta > DELTA_LIMITE ? DELTA_LIMITE : delta);

		move(BASE_POWER, delta);

	
		int floorTag = getFloorTag();
		if(floorTag == CIRCLE_TAG) {
			return WAIT;
		}
		
		return RETURN_Q4;
		
	}
	
	private static int centerState() {

		switch (mFlameSensor.scan()) { // Make a scan to get the direction towards the flame.

		case 1: // The flame is far to the left.
			rotate(-3);
			break;
		case 2: // The flame is slightly to the left.
			rotate(-3);
			break;
		case 3: // The flame is ahead.
			move(8, 0); // Go forward slowly towards the flame.
			if (getDistance(mFrontSonar) < 15) {
				stop();
				return PUT_OUT;
				// return WAIT;
			}
			break;
		case 4: // The flame is slightly to the right.
			rotate(3);
			break;
		case 5: // The flame is far to the right.
			rotate(3); // Turn far left
			break;
		case -1: // The flame was lost.
			// rotate(-3);
			break;
		}

		displayFlameSensorData(1);

		return CENTER;
	}

	private static int putOutState() {
		mFan.setPower(16);
		wait(2000);
		mFan.setPower(0);
		mFlameLED.clear();
		
		
		maneuverPutOut();
		
		
		return RETURN;
	}

	private static int returnState() {
		
		
		switch(mRoom) {
		case 1:
			return NAVIGATE_LEFT;
		case 2:
			return NAVIGATE_BACK;
		case 3:
			return RETURN_Q3;
		case 4: 
			return NAVIGATE_RIGHT;
		}
		
		
		
		/*if (getDistance(mFrontSonar) < MIN_DISTANCE_FRONT)
			rotateAngle(90);

		checkBumpers();
		int delta = (int) ((getDistance(mRightSonar) - MIN_DISTANCE_RIGHT) * GAIN);

		delta = (delta > DELTA_LIMITE ? DELTA_LIMITE : delta);

		move(BASE_POWER, delta);

		//if (mLineSensor.sample() < LINE_LIMIT) {
			//return WAIT; // White circle detected.
		//}
	
		int floorTag = getFloorTag();
		if(floorTag == CIRCLE_TAG) {
			return WAIT;
		}else if(mTotalLines == 2) {
			stop();
			//dentroQuarto = false;
		}else if (floorTag == LINE_TAG && dentroQuarto == false) {
			rotateAngle(90);
			
		}*/
		

		return WAIT;
	}

	
	// ================================================================================
	// Methods to read sensors.
	// ================================================================================

	private static float getDistance(RangeFinder s) {
		s.ping();
		wait(10);
		float d = s.getDistanceCm();
		return (d < 0 ? 100f : d);
	}

	private static void checkBumpers() {
		if (!mLeftBumper.isSet() && !mRightBumper.isSet()) {
			move(-BASE_POWER, 0); // Move backwards.
			wait(1000);
		} else if (!mLeftBumper.isSet()) {
			move(-BASE_POWER, 5); // Move backwards to the left.
			wait(1000);
		} else if (!mRightBumper.isSet()) {
			move(-BASE_POWER, -5); // Move backwards to the right.
			wait(1000);
		}
	}

	private static void countLines() {
		if (mLineSensor.sample() < LINE_LIMIT) {
			move(BASE_POWER, 0);
			wait(500);
			mTotalLines++;
			mLcd.print(1, "L: " + mTotalLines);
		}
	}

	private static void countLines2() {
		if (getFloorTag() == LINE_TAG) {
			mTotalLines++;
			mLcd.print(1, "L: " + mTotalLines);
		}
	}

	private static void testLineSensor() {
		mLcd.print(1, "L: " + mLineSensor.sample());
	}

	private static int getFloorTag() {
		if (mLineSensor.sample() < LINE_LIMIT) {
			move(BASE_POWER, 0);
			wait(500);
			// stop();
			if (mLineSensor.sample() < LINE_LIMIT)
				return CIRCLE_TAG; // White circle detected.
			else
				return LINE_TAG; // White line detected.
		}
		return NO_TAG; // No white tag detected.
	}

	private static int checkStopButton(int s) {
		if (!mStopButton.isSet()) {
			stop();
			return WAIT;
		}
		return s;
	}

	// ================================================================================
	// Methods to move the robot.
	// ================================================================================

	private static void move(int power, int delta) {
		mLeftMotor.setPower(power + delta);
		mRightMotor.setPower(power - delta);
	}

	private static void rotate(int power) {
		mLeftMotor.setPower(-power);
		mRightMotor.setPower(power);
	}

	private static void rotateAngle(int angle) {
		if (angle < 0) {
			angle = -angle;
			rotate(-ROTATE_POWER);
		} else {
			rotate(ROTATE_POWER);
		}

		wait((int) (angle * ANGLE_TO_TIME_FACTOR));
		stop();
	}

	private static void stop() {
		mLeftMotor.stop();
		mRightMotor.stop();
	}

	private static void brake() {
		mLeftMotor.brake();
		mRightMotor.brake();
	}

	// ================================================================================
	// Methods to implement maneuvers.
	// ================================================================================

	private static void maneuverPutOut() {
		
		rotateAngle(-90);
		do {
		move(BASE_POWER,0);
		}while((getDistance(mFrontSonar) > MIN_DISTANCE_FRONT) && (getDistance(mRightSonar) > MIN_DISTANCE_RIGHT));
	}
	
	private static void maneuverToExitWhiteCircle() {
		move(Motor.MAX_FORWARD, 0);
		do {
		} while (mLineSensor.sample() < LINE_LIMIT);
	}

	private static void maneuverToReturnBack() {
		rotateAngle(180);
		move(BASE_POWER, 0);
		wait(2000);
	}

	private static void maneuverToCorrectDirection() {
		if (getDistance(mLeftSonar) < 30)
			do
				rotate(ROTATE_POWER);
			while (getDistance(mRightSonar) > 20);
		// rotateAngle(-90);
	}

	private static void maneuverToGoToNextRoom() {
		switch (mRoom) {
		case 1:
			rotateAngle(180);
			do
				move(BASE_POWER, 0);
			while (getDistance(mFrontSonar) > 15);
			// wait(2000);
			break;
		case 2:
			rotateAngle(180);
			move(BASE_POWER, 3);
			wait(2000);
			break;
		case 3:
			break;
		case 4:
			break;
		}
	}
	// ================================================================================
	// Auxiliary Methods
	// ================================================================================

	
	
	private static void wait(int ms) {
		try {
			Thread.sleep(ms);
		} catch (Throwable t) {
			t.printStackTrace();
		}
	}

	private static void displayState(int s) {
		switch (s) {
		case WAIT:
			mLcd.print(0, "WAIT");
			break;
		case NAVIGATE_RIGHT:
			mLcd.print(0, "NAV RIGHT");
			break;
		case CENTER:
			mLcd.print(0, "CENTER");
			break;
		case PUT_OUT:
			mLcd.print(0, "PUT OUT");
			break;
		}
	}

	private static void displayFlameSensorData(int l) {
		mLcd.print(l,
				"V:" + mFlameSensor.getValue() + "(" + IS_PRESENT_LIMIT + ")" + " D:" + mFlameSensor.getDirection());
	}

}
