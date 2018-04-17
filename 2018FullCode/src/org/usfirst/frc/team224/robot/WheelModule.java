package org.usfirst.frc.team224.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.TalonSRX.FeedbackDevice;
//import com.ctre.TalonSRX.TalonControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;

public class WheelModule {

	private WPI_TalonSRX steer;// motor to steer.
	private WPI_TalonSRX drive;// motor to drive.
	private int whichWheel;// number of the wheel
	private int invert;
	private int sideInverted;
	PIDController controller;

	/*
	 * right side has the motor direction inverted, so this has to be -1 for the
	 * right side.
	 */

	public WheelModule(WPI_TalonSRX steer, WPI_TalonSRX drive, int whichModule) {

		// steer.changeControlMode(TalonControlMode.Position);
		// steer.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);
		steer.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		// steer.reverseSensor(true);
		steer.setSensorPhase(true);
		// drive.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		drive.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

		// steer.setPID(1, 0, 0);
		steer.config_kP(0, 1, 0);
		steer.config_kI(0, 0, 0);
		steer.config_kD(0, 0, 0);
		drive.config_kP(0, Constants.driveP, 0);
		drive.config_kI(0, Constants.driveI, 0);
		drive.config_kD(0, Constants.driveD, 0);
		drive.configAllowableClosedloopError(0, (int) (1 / Constants.driveWheelRatio / 2 * Constants.encoderTicks), 0);
		this.steer = steer;
		if (whichModule > 1) {
			sideInverted = 1;
		} else {
			sideInverted = -1;
		}
		this.drive = drive;
		whichWheel = whichModule;
		if (steerConvertToDegrees(steer.getSelectedSensorPosition(0)) < 90
				|| steerConvertToDegrees(steer.getSelectedSensorPosition(0)) > 270) {
			invert = 1;
		} else {
			invert = -1;
		}
	}

	public void drive(double inputAngle, double speed) {
		// steer.setPosition(steer.getPosition()%3);
		double newAngle;
		double currentAngle = steerConvertToDegrees(steer.getSelectedSensorPosition(0));
		double invertedAngle = (inputAngle + 180) % 360;
		if (Math.abs(Math.IEEEremainder(inputAngle - currentAngle, 360)) < Math
				.abs(Math.IEEEremainder(invertedAngle - currentAngle, 360))) {

			newAngle = currentAngle + Math.IEEEremainder(inputAngle - currentAngle, 360);
			invert = 1;
		} else {
			invert = -1;
			newAngle = currentAngle + Math.IEEEremainder(invertedAngle - currentAngle, 360);
		}
		// steer.set(convertToEncoderUnits(newAngle));
		steer.set(ControlMode.Position, steerConvertToEncoderUnits(newAngle));
		// if (drive.isEnabled())
		// drive.set(speed * invert * sideInverted);
		drive.set(ControlMode.PercentOutput, speed * invert * sideInverted);
	}

	public void driveDistance(double encoderDistance, double angle) {
		drive(0, 0);
		controller.setSetpoint(0);// SET THE VELOCITY SETPOINT
		drive.set(ControlMode.Position, encoderDistance * getInverted());
		drive.set(ControlMode.Position, encoderDistance);
		// drive.set(mode, demand0, demand1);
		drive.getClosedLoopError(0);
	}

	public double steerConvertToDegrees(double angle) {
		// based on ratio
		return angle * 120;
	}

	public double steerConvertToEncoderUnits(double angle) {
		// based on ratio
		return angle / 360 * Constants.steerWheelRatio * Constants.encoderTicks;

	}

	public void disableDrive() {
		drive.disable();
	}

	public void disableSteer() {
		steer.disable();
	}

	public void disableBoth() {
		disableSteer();
		disableDrive();
	}

	// public void enableDrive() {
	// drive.enable();
	// }
	//
	// public void enableSteer() {
	// steer.enable();
	// }

	// public void enableBoth() {
	// enableDrive();
	// enableSteer();
	// }
	public void print() {
		SmartDashboard.putNumber("Wheel " + whichWheel + " angle:",
				(/* steer.getPosition() */steer.getSelectedSensorPosition(0) * 120) % 360);
	}

	public void resetDriveEncPos() {
		// drive.setPosition(0);
		drive.setSelectedSensorPosition(0, 0, 0);
	}

	public double getDriveEncPos() {
		// return drive.getPosition();
		return drive.getSelectedSensorPosition(0);
	}

	public double getSteerEncPos() {
		// return steer.getPosition();
		return steer.getSelectedSensorPosition(0);
	}

	public void matchStartup() {
		// steer.setPosition(steer.getPosition() % 3);
		// this.drive(0, 0);
	}

	public void resetSteerEncPos() {
		// steer.setPosition(0);
		steer.setSelectedSensorPosition(0, 0, 0);
	}

	private int getInverted() {
		if (invert * sideInverted > 0) {
			return 1;
		}
		return -1;

	}
}