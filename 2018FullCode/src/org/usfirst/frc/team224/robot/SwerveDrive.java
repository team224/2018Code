package org.usfirst.frc.team224.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {
	WheelModule frontLeft;
	WheelModule backLeft;
	WheelModule frontRight;
	WheelModule backRight;
	Joystick joy;
	double fwd, str, rcw;
	double l, w, r;
	AHRS navx;
	double a, b, c, d;
	double ws0, ws1, ws2, ws3;
	double wa0, wa1, wa2, wa3;
	double throttle;

	public SwerveDrive(WPI_TalonSRX fls, WPI_TalonSRX fld, WPI_TalonSRX bls, WPI_TalonSRX bld, WPI_TalonSRX frs,
			WPI_TalonSRX frd, WPI_TalonSRX brs, WPI_TalonSRX brd, Joystick joy) {
		frontLeft = new WheelModule(fls, fld, 0);
		backLeft = new WheelModule(bls, bld, 1);
		frontRight = new WheelModule(frs, frd, 2);
		backRight = new WheelModule(brs, brd, 3);
		navx = new AHRS(SerialPort.Port.kMXP);
		this.joy = joy;
		l = Constants.wheelbase;
		w = Constants.trackwidth;
		r = Math.sqrt(Math.pow(l, 2) + Math.pow(w, 2));
	}

	public void drive() {
		calculations();
		if ((joy.getMagnitude() > .2 || Math.abs(joy.getTwist()) > .1) && joy.getPOV() == -1) {
			frontLeft.drive(wa0, ws0 * throttle);
			backLeft.drive(wa1, ws1 * throttle);
			frontRight.drive(wa2, ws2 * throttle);
			backRight.drive(wa3, ws3 * throttle);
		} else if (joy.getPOV() == 0) {
			frontLeft.drive(0, throttle);
			backLeft.drive(0, throttle);
			frontRight.drive(0, throttle);
			backRight.drive(0, throttle);
		} else if (joy.getPOV() == 180) {
			frontLeft.drive(180, throttle);
			backLeft.drive(180, throttle);
			frontRight.drive(180, throttle);
			backRight.drive(180, throttle);
		} else if (joy.getPOV() == 90) {
			frontLeft.drive(90, throttle);
			backLeft.drive(90, throttle);
			frontRight.drive(90, throttle);
			backRight.drive(90, throttle);
		} else if (joy.getPOV() == 270) {
			frontLeft.drive(270, 0);
			backLeft.drive(270, throttle);
			frontRight.drive(270, throttle);
			backRight.drive(270, throttle);
		} else {
			frontLeft.drive(0, 0);
			backLeft.drive(0, 0);
			frontRight.drive(0, 0);
			backRight.drive(0, 0);
		}
	}

	public void driveDistance2(double ft, double inches) {
		double totalDistance = ft * 12 + inches;
		double wheelCircumference = Constants.wheelDiameter * Math.PI;
		double wheelRevolutions = totalDistance / wheelCircumference;
		double driveEnc1WheelRevolution = 1 / Constants.driveWheelRatio * Constants.encoderTicks;
		double totalEncoderDistance = wheelRevolutions * driveEnc1WheelRevolution;
		frontLeft.driveDistance(totalEncoderDistance, 0);
		backLeft.driveDistance(totalEncoderDistance, 0);
		frontRight.driveDistance(totalEncoderDistance, 0);
		backRight.driveDistance(totalEncoderDistance, 0);

	}

	@Deprecated
	public void driveDistance(double ft, double inches, double degree) {
		double encoderDistance = circumEq(ft, inches);
		final double DEFAULT_SPEED = .3;
		frontLeft.resetDriveEncPos();
		backLeft.resetDriveEncPos();
		frontRight.resetDriveEncPos();
		backRight.resetDriveEncPos();
		if (navx.getAngle() > 2.5) {
			if (Math.abs(frontLeft.getDriveEncPos()) < encoderDistance - 1) {
				frontLeft.drive(degree, DEFAULT_SPEED * .9);
			}
			if (Math.abs(backLeft.getDriveEncPos()) < encoderDistance - 1) {
				backLeft.drive(degree, DEFAULT_SPEED * .9);
			}
			if (Math.abs(frontRight.getDriveEncPos()) < encoderDistance - 1) {
				frontRight.drive(degree, DEFAULT_SPEED * 1.1);
			}
			if (Math.abs(backRight.getDriveEncPos()) < encoderDistance - 1) {
				backRight.drive(degree, DEFAULT_SPEED * 1.1);
			}

		} else if (navx.getAngle() < -2.5) {
			if (Math.abs(frontLeft.getDriveEncPos()) < encoderDistance - 1) {
				frontLeft.drive(degree, DEFAULT_SPEED * 1.1);
			}
			if (Math.abs(backLeft.getDriveEncPos()) < encoderDistance - 1) {
				backLeft.drive(degree, DEFAULT_SPEED * 1.1);
			}
			if (Math.abs(frontRight.getDriveEncPos()) < encoderDistance - 1) {
				frontRight.drive(degree, DEFAULT_SPEED * .9);
			}
			if (Math.abs(backRight.getDriveEncPos()) < encoderDistance - 1) {
				backRight.drive(degree, DEFAULT_SPEED * .9);
			}
		} else {
			if (Math.abs(frontLeft.getDriveEncPos()) < encoderDistance - 1) {
				frontLeft.drive(degree, DEFAULT_SPEED);
			}
			if (Math.abs(backLeft.getDriveEncPos()) < encoderDistance - 1) {
				backLeft.drive(degree, DEFAULT_SPEED);
			}
			if (Math.abs(frontRight.getDriveEncPos()) < encoderDistance - 1) {
				frontRight.drive(degree, DEFAULT_SPEED);
			}
			if (Math.abs(backRight.getDriveEncPos()) < encoderDistance) {
				backRight.drive(degree, DEFAULT_SPEED);
			}
		}
	}

	private double circumEq(double feet, double inches) {
		return (((feet * 12.0) + inches) / (Constants.wheelDiameter * Math.PI)) * 3.2;
	}

	/**
	 * Rotates the robot
	 * 
	 * @param degrees
	 */
	public void rotateTo(double degrees) {

	}

	public void ratateDegrees(double degrees) {
	}

	public void calculations() {
		throttle = convertedThrottle();
		fwd = joy.getY() * -1;
		str = joy.getX();
		rcw = joy.getTwist();
		if (!joy.getTrigger()) {
			double temp = fwd * Math.cos(navx.getAngle() * (Math.PI / 180))
					+ str * Math.sin(navx.getAngle() * (Math.PI / 180));
			str = -fwd * Math.sin(navx.getAngle() * (Math.PI / 180))
					+ str * Math.cos(navx.getAngle() * (Math.PI / 180));
			fwd = temp;
		}
		a = fwd - rcw * (l / r);
		b = fwd + rcw * (l / r);
		c = fwd - rcw * (w / r);
		d = fwd + rcw * (w / r);
		ws0 = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2)); // front left
		wa0 = Math.atan2(b, d) * 180 / Math.PI;// front left

		ws1 = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2)); // rear left
		wa1 = Math.atan2(a, d) * 180 / Math.PI;// rear left

		ws2 = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));// front right
		wa2 = Math.atan2(b, c) * 180 / Math.PI;// front right

		ws3 = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2)); // rear right
		wa3 = Math.atan2(a, c) * 180 / Math.PI; // rear right
		double max = Math.max(Math.max(ws0, ws1), Math.max(ws2, ws3));
		ws0 /= max;
		ws1 /= max;
		ws2 /= max;
		ws3 /= max;
	}

	private double convertedThrottle() {
		return joy.getThrottle() / -2 + .5;
	}

	@Deprecated
	public void matchStart() {
		frontLeft.matchStartup();
		backLeft.matchStartup();
		frontRight.matchStartup();
		backRight.matchStartup();
	}

	@Deprecated
	public void resetEncoders() {
		frontLeft.resetSteerEncPos();
		backLeft.resetSteerEncPos();
		frontRight.resetSteerEncPos();
		backRight.resetSteerEncPos();
	}

	public void print() {
		SmartDashboard.putNumber("FrontLeft", frontLeft.getSteerEncPos());
		SmartDashboard.putNumber("BackLeft", backLeft.getSteerEncPos());
		SmartDashboard.putNumber("FrontRight", frontRight.getSteerEncPos());
		SmartDashboard.putNumber("BackRight", backRight.getSteerEncPos());
	}
}