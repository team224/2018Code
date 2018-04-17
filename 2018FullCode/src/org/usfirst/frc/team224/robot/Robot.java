/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team224.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	// Declarations for autonomous choosers
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private static final String driveForward = "Drive Forward";
	private static final String centerDriveSwitch = "Center Drive Switch";
	private String autoSelected;
	private SendableChooser<String> chooser = new SendableChooser<>();
	private String gameData;
	// Declarations for swerve
	// Key: front/back,left/right,drive/steer
	WPI_TalonSRX fld, fls;
	WPI_TalonSRX bld, bls;
	WPI_TalonSRX frd, frs;
	WPI_TalonSRX brd, brs;
	SwerveDrive swerve;

	// Declarations for BoxSubsystem
	WPI_TalonSRX elevator;
	DigitalInput upperLS, lowerLS;
	DoubleSolenoid pusher, grabber;
	Timer armTimer;
	private int armPosition;

	// Declarations for ramp

	// Other Declarations/multiSystem declarations
	Compressor compress;
	Joystick drive, aux;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		boxSubsystemInit();
		swerveMotorsInit();
		joyInit();
		rampInit();
		swerve = new SwerveDrive(fls, fld, bls, bld, frs, frd, brs, brs, drive);
		compress = new Compressor();
		chooser.addDefault("Drive Forward", driveForward);
		chooser.addObject("Center Drive Switch", centerDriveSwitch);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case driveForward:
			// Put custom auto code here
			break;
		case kDefaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	public void teleopInit() {
		grabber.set(Value.kReverse);
		pusher.set(Value.kForward);
		armTimer.start();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		swerve.drive();
		//elevator();
		//grabber();
		//ramp();

	}

	public void elevator() {

		if (aux.getY() > .1 && lowerLS.get()) {
			elevator.set(ControlMode.PercentOutput, aux.getY());
		} else if (aux.getY() < -.1 && upperLS.get()) {
			elevator.set(ControlMode.PercentOutput, aux.getY());
		} else {
			elevator.set(ControlMode.PercentOutput, 0);
		}
	}

	public void grabber() {
		if (aux.getRawButton(4) && armTimer.get() > .3) {
			switch (armPosition) {
			case 0:
				grabber.set(Value.kReverse);
				pusher.set(Value.kForward);
				armPosition++;
				break;
			case 1:
				grabber.set(Value.kForward);
				pusher.set(Value.kReverse);
				armPosition++;
				break;
			case 2:
				pusher.set(Value.kForward);
				armPosition++;
				break;
			case 3:
				grabber.set(Value.kReverse);
				armPosition = 0;
				break;
			}
			armTimer.reset();
		}
	}

	public void ramp() {

	}

	public void swerveMotorsInit() {
		fld = new WPI_TalonSRX(1);
		fls = new WPI_TalonSRX(2);
		bld = new WPI_TalonSRX(3);
		bls = new WPI_TalonSRX(4);
		frd = new WPI_TalonSRX(5);
		frs = new WPI_TalonSRX(7);
		brd = new WPI_TalonSRX(6);
		brs = new WPI_TalonSRX(8);
	}

	public void joyInit() {
		drive = new Joystick(0);
		aux = new Joystick(1);
	}

	public void elevatorInit() {
		elevator = new WPI_TalonSRX(20);
		upperLS = new DigitalInput(0);
		lowerLS = new DigitalInput(1);
	}

	public void armsInit() {
		grabber = new DoubleSolenoid(0, 1);
		pusher = new DoubleSolenoid(1, 2);
		armTimer = new Timer();
		armPosition = 0;
	}

	public void boxSubsystemInit() {
		elevatorInit();
		armsInit();
	}

	public void rampInit() {
	}
}