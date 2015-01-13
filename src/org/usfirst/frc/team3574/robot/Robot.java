package org.usfirst.frc.team3574.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.nav6.frc.IMU; 
import com.kauailabs.nav6.frc.IMUAdvanced;

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
	int autoLoopCounter;
	
    SerialPort serial_port;
    //IMU imu;  // Alternatively, use IMUAdvanced for advanced features
    IMUAdvanced imu;
    boolean first_iteration;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	myRobot = new RobotDrive(0,1);
    	stick = new Joystick(0);

    	try {
    	serial_port = new SerialPort(57600,SerialPort.Port.kUSB);
		
		// You can add a second parameter to modify the 
		// update rate (in hz) from 4 to 100.  The default is 100.
		// If you need to minimize CPU load, you can set it to a
		// lower value, as shown here, depending upon your needs.
		
		// You can also use the IMUAdvanced class for advanced
		// features.
		
		byte update_rate_hz = 50;
		//imu = new IMU(serial_port,update_rate_hz);
		imu = new IMUAdvanced(serial_port,update_rate_hz);
    	} catch( Exception ex ) {
    		System.out.println(ex.toString());
    	}
        if ( imu != null ) {
            LiveWindow.addSensor("IMU", "Gyro", imu);
        }
        first_iteration = true;
}
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
			} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
		}
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        myRobot.arcadeDrive(stick);

        // When calibration has completed, zero the yaw
        // Calibration is complete approaximately 20 seconds
        // after the robot is powered on.  During calibration,
        // the robot should be still
        
        System.out.println((imu == null));
        
        boolean is_calibrating = imu.isCalibrating();
        if ( first_iteration && !is_calibrating ) {
            Timer.delay( 0.3 );
            imu.zeroYaw();
            first_iteration = false;
        }
        
        // Update the dashboard with status and orientation
        // data from the nav6 IMU
        
        SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              imu.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
        SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
        SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
        SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());

        // If you are using the IMUAdvanced class, you can also access the following
        // additional functions, at the expense of some extra processing
        // that occurs on the CRio processor
        
        SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
        SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
        SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());
        
        Timer.delay(0.2);
}
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
