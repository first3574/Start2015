package org.usfirst.frc.team3574.robot;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	JoystickButton leftBumper;
	JoystickButton rightBumper;
	double deadZone;
	int autoLoopCounter;
	CANTalon backLeftMotor;
	CANTalon backRightMotor;
	CANTalon frontLeftMotor;
	CANTalon frontRightMotor;
	CANTalon elevatorMotor;
	CANTalon motor6;
	CANTalon motor7;
	static final int[] stopPoints= new int[]{10,200,500};
	boolean amMoving;
	int lastBand;

	double scaledX, scaledY, scaledZ, rookieFactor;
	// make the imu be 0 to 360
	double forceToBetotwoPi = 0.0;

	SerialPort serial_port;
	// IMU imu; // Alternatively, use IMUAdvanced for advanced features
	IMUAdvanced imu;
	boolean first_iteration;
	
	public void talonInit(CANTalon theThing) {
		theThing.changeControlMode(CANTalon.ControlMode.Speed);
		theThing.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		theThing.enableBrakeMode(true);
		theThing.setPID(1.0, 0.0, 0.0, 3.95, 0, 0.0, 0);
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		myRobot = new RobotDrive(4, 1, 3, 2);
		stick = new Joystick(0);
		leftBumper = new JoystickButton(stick, 10);
		rightBumper = new JoystickButton(stick, 11);
		backLeftMotor = new CANTalon(2);
		backRightMotor = new CANTalon(4);
		frontLeftMotor = new CANTalon(1);
		frontRightMotor = new CANTalon(3);
		elevatorMotor = new CANTalon(5);
		motor6 = new CANTalon(6);
		motor7 = new CANTalon(7);
		amMoving = false;
		lastBand = -1;
		
		talonInit(backLeftMotor);
		talonInit(frontLeftMotor);
		talonInit(backRightMotor);
		talonInit(frontRightMotor);
		
		try {
			serial_port = new SerialPort(57600, SerialPort.Port.kUSB);

			// You can add a second parameter to modify the
			// update rate (in hz) from 4 to 100. The default is 100.
			// If you need to minimize CPU load, you can set it to a
			// lower value, as shown here, depending upon your needs.

			// You can also use the IMUAdvanced class for advanced
			// features.

			byte update_rate_hz = 100;
			// imu = new IMU(serial_port,update_rate_hz);
			imu = new IMUAdvanced(serial_port, update_rate_hz);
		} catch (Exception ex) {
			System.out.println(ex.toString());
		}
		if (imu != null) {
			LiveWindow.addSensor("IMU", "Gyro", imu);
		}
		first_iteration = true;
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		autoLoopCounter = 0;
		frontLeftMotor.setPosition(0);
		frontRightMotor.setPosition(0);
		backLeftMotor.setPosition(0);
		backRightMotor.setPosition(0);
	}

	double tryToGoStartRight = .5;
	double tryToGoStartLeft = .5;

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

		if (autoLoopCounter < 100) // Check if we've completed 100 loops
									// (approximately 2 seconds)
		{
			SmartDashboard.putNumber("Left Front enc",
					frontLeftMotor.getEncPosition());
			SmartDashboard.putNumber("Right Front enc",
					frontRightMotor.getEncPosition());
			SmartDashboard.putNumber("Right Back enc",
					backRightMotor.getEncPosition());
			SmartDashboard.putNumber("Left Back enc",
					backLeftMotor.getEncPosition());

			if (frontLeftMotor.getEncPosition() + 10 > frontRightMotor
					.getEncPosition()) {
				tryToGoStartRight -= .1;
			} else if (frontLeftMotor.getEncPosition() - 10 < frontRightMotor
					.getEncPosition()) {
				tryToGoStartLeft -= .1;
			}

			if (backLeftMotor.getEncPosition() + 10 > backRightMotor
					.getEncPosition()) {
				tryToGoStartRight -= .1;
			} else if (backLeftMotor.getEncPosition() - 10 < backRightMotor
					.getEncPosition()) {
				tryToGoStartLeft -= .1;
			}

			backLeftMotor.set(-tryToGoStartLeft); // drive forwards half speed
			backRightMotor.set(tryToGoStartRight);
			frontLeftMotor.set(-tryToGoStartLeft);
			frontRightMotor.set(tryToGoStartRight);
			autoLoopCounter++;

		} else {
			backLeftMotor.set(-0); // drive forwards half speed
			backRightMotor.set(0);
			frontLeftMotor.set(-0);
			frontRightMotor.set(0);
		}

	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	public void teleopInit() {
		imu.zeroYaw();
	}

	public double joystickScale(double input) {
		boolean isNegative = false;
		if (input < 0.0) {
			isNegative = true;
		}
		input = Math.abs(input);
		double result = (Math.pow(Math.E, input) - 1) / 1.718;
		if (isNegative) {
			result *= -1.0;
		}
		return result;
	}

	
	boolean isInRangeBand(int pos) {
		if(getElevatorPos()>= pos-10 && getElevatorPos() <= pos+10)
			return true;
		return false;
	}
	
	int whichBand(){
		for(int stopPoint : stopPoints) {
			if(isInRangeBand(stopPoint)) {
				return stopPoint;
			}
		}
		return -1;
	}
	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		rookieFactor = 1.0;
		deadZone = 0.14;
		// To make it field oriented the x needs to be inverted
//		scaledX = joystickScale(-stick.getRawAxis(0)) * rookieFactor;
//		scaledY = joystickScale(stick.getRawAxis(1)) * rookieFactor * -1.0;
//		scaledZ = joystickScale(stick.getRawAxis(4)) * rookieFactor;
		scaledX = Math.abs(-stick.getRawAxis(0)) < deadZone ? 0.0 : -stick.getRawAxis(0) * rookieFactor;
		scaledY = Math.abs(-stick.getRawAxis(1)) < deadZone ? 0.0 : -stick.getRawAxis(1) * rookieFactor; 
		scaledZ = Math.abs(stick.getRawAxis(3) - stick.getRawAxis(2)) < deadZone ? 0.0 : stick.getRawAxis(3) - stick.getRawAxis(2) * rookieFactor;
		scaledX = joystickScale(scaledX);
		scaledY = joystickScale(scaledY);				
		scaledZ = joystickScale(scaledZ);
		
		if (imu.getYaw() < 0.0) {
			forceToBetotwoPi = 360 + imu.getYaw();
		} else {
			forceToBetotwoPi = imu.getYaw();
		}



		mecanumDrive_Cartesian(scaledX, scaledY, scaledZ, forceToBetotwoPi);
		// mecanumDrive_Cartesian(scaledX, scaledY, scaledZ, 0.0);

		int bumperPressed = 0;  //0 is no bumper, -1 is left, 1 is right		
		if (stick.getRawButton(5)) {
			bumperPressed = -1;
		} else if(stick.getRawButton(6)) {
			bumperPressed = 1;
		}
		if (Math.abs(bumperPressed ) == 1) {
			if (whichBand() == -1 || lastBand == whichBand()){
					elevatorMotor.set(.5*bumperPressed);
					lastBand = whichBand();
			}
		}
		
		else if (whichBand() != -1 && whichBand() != lastBand){
			elevatorMotor.set(0);
			lastBand = whichBand();
		}
		
		// myRobot.arcadeDrive(stick);
		// mecanumDrive_Cartesian(stick.getRawAxis(0), stick.getRawAxis(1),
		// stick.getRawAxis(4), 0.0);

		// motor1.set(stick.getRawAxis(0));
		// motor2.set(stick.getRawAxis(1));
		//
		// if (stick.getRawButton(4) == true){
		// motor7.set(0.3);
		// } else{
		// motor7.set(0.0);
		// }

		// When calibration has completed, zero the yaw
		// Calibration is complete approximately 20 seconds
		// after the robot is powered on. During calibration,
		// the robot should be still

		// System.out.println((imu == null));

		boolean is_calibrating = imu.isCalibrating();
		if (first_iteration && !is_calibrating) {
			Timer.delay(0.3);
			imu.zeroYaw();
			first_iteration = false;
		}

		// Update the dashboard with status and orientation
		// data from the nav6 IMU
		
		SmartDashboard.putNumber("AnalogInRaw", elevatorMotor.getAnalogInRaw());
		SmartDashboard.putNumber("LastBand", lastBand);

		SmartDashboard.putNumber("X_Axis", stick.getRawAxis(0));
		SmartDashboard.putNumber("Y_Axis", stick.getRawAxis(1));

		SmartDashboard.putBoolean("IMU_Connected", imu.isConnected());
		SmartDashboard.putBoolean("IMU_IsCalibrating", imu.isCalibrating());
		SmartDashboard.putNumber("IMU_Yaw", imu.getYaw());
		SmartDashboard.putNumber("IMU_Pitch", imu.getPitch());
		SmartDashboard.putNumber("IMU_Roll", imu.getRoll());
		SmartDashboard.putNumber("IMU_CompassHeading", imu.getCompassHeading());
		SmartDashboard.putNumber("IMU_Update_Count", imu.getUpdateCount());
		SmartDashboard.putNumber("IMU_Byte_Count", imu.getByteCount());
		SmartDashboard.putNumber("IMU_PIDGET", imu.pidGet());

		SmartDashboard.putNumber("frontLeftMotor voltage", frontLeftMotor.getEncVelocity());
		SmartDashboard.putNumber("frontRighttMotor voltage", frontRightMotor.getEncVelocity());
		SmartDashboard.putNumber("backLeftMotor voltage", backLeftMotor.getEncVelocity());
		SmartDashboard.putNumber("backRighttMotor voltage", backRightMotor.getEncVelocity());

		
		SmartDashboard
				.putNumber("Left enc TP", frontLeftMotor.getEncPosition());
		SmartDashboard.putNumber("Right enc TP",
				frontRightMotor.getEncPosition());

		// imu.pidGet()

		// If you are using the IMUAdvanced class, you can also access the
		// following
		// additional functions, at the expense of some extra processing
		// that occurs on the CRio processor

		SmartDashboard.putNumber("IMU_Accel_X", imu.getWorldLinearAccelX());
		SmartDashboard.putNumber("IMU_Accel_Y", imu.getWorldLinearAccelY());
		SmartDashboard.putBoolean("IMU_IsMoving", imu.isMoving());
		SmartDashboard.putNumber("IMU_Temp_C", imu.getTempC());
		SmartDashboard.putNumber("forceToBetotwoPi", forceToBetotwoPi);

		// SmartDashboard.putNumber( "Motor 1", motor1.getEncPosition());
		// SmartDashboard.putNumber( "Motor 4 Encoder count",
		// motor4.getEncPosition());
		SmartDashboard.putBoolean("motor5 limitswitch forward",
				elevatorMotor.isFwdLimitSwitchClosed());
		SmartDashboard.putBoolean("motor5 limitswitch back",
				elevatorMotor.isRevLimitSwitchClosed());

		Timer.delay(0.007);
	}

	public int getElevatorPos() {
		return elevatorMotor.getAnalogInRaw();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}

	protected static double[] rotateVector(double x, double y, double angle) {
		double cosA = Math.cos(angle * (3.14159 / 180.0));
		double sinA = Math.sin(angle * (3.14159 / 180.0));
		double out[] = new double[2];
		out[0] = x * cosA - y * sinA;
		out[1] = x * sinA + y * cosA;
		return out;
	}

	protected static void normalize(double wheelSpeeds[]) {
		double maxMagnitude = Math.abs(wheelSpeeds[0]);
		int i;
		for (i = 1; i < 4; i++) {
			double temp = Math.abs(wheelSpeeds[i]);
			if (maxMagnitude < temp)
				maxMagnitude = temp;
		}
		if (maxMagnitude > 1.0) {
			for (i = 0; i < 4; i++) {
				wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
			}
		}
	}

	public void mecanumDrive_Cartesian(double x, double y, double rotation,
			double gyroAngle) {

		double xIn = x;
		double yIn = y;
		// Negate y for the joystick.
		yIn = -yIn;
		// Compenstate for gyro angle.
		double rotated[] = rotateVector(xIn, yIn, gyroAngle);
		xIn = rotated[0];
		yIn = rotated[1];

		double wheelSpeeds[] = new double[4];
		wheelSpeeds[0] = xIn + yIn + rotation;
		wheelSpeeds[1] = -xIn + yIn - rotation;
		wheelSpeeds[2] = -xIn + yIn + rotation;
		wheelSpeeds[3] = xIn + yIn - rotation;

		normalize(wheelSpeeds);

		frontLeftMotor.set(wheelSpeeds[0] * -275.0);
		frontRightMotor.set(wheelSpeeds[1] * 275.0);
		backLeftMotor.set(wheelSpeeds[2] * -275.0);
		backRightMotor.set(wheelSpeeds[3] * 275.0);

	}

	public void driveForwordX(double frontRight, double backRight,
			double forntLeft, double backLeft, int x) {

	}

}
