package org.usfirst.frc.team4392.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {

	private TalonSRX left1, left2, left3, right1, right2, right3;
	private PigeonIMU pidgey;
	private Solenoid shifter = new Solenoid(0);
	
	private double kpAngle = 0.03;
	private double kdAngle = 0.004;
	
	private static int encoderOffsetRight = 0;
	private static int encoderOffsetLeft = 0;
	private static double angleOffset = 0;
	
	private static double diameter = 6;
	private static double circumference = Math.PI * diameter;
	private static double countsPerRot = 4096;
	private static double distancePerCount = circumference/countsPerRot;
	private static double countsPerDistance = countsPerRot/circumference;
	
	public Drivetrain(){
		 left1 = new TalonSRX(1);
		 left2 = new TalonSRX(2);
		 left3 = new TalonSRX(3);
		 right1 = new TalonSRX(12);
		 right2 = new TalonSRX(11);
		 right3 = new TalonSRX(10);
		 
		 pidgey = new PigeonIMU(right2);
		 
		 
		 left1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		 left1.setSensorPhase(false);
		 left1.setInverted(false);
		 left1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		 left1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		 left1.configNominalOutputForward(0, 0);
		 left1.configNominalOutputReverse(0, 0);
		 left1.configPeakOutputForward(1, 0);
		 left1.configPeakOutputReverse(-1, 0);
		 left1.selectProfileSlot(0, 0);
		 left1.config_kF(0, .5, 0);
		 left1.config_kP(0, 0, 0);
		 left1.configMotionCruiseVelocity(2500, 0);
		 left1.configMotionAcceleration(((int)(2500/3)), 0);
		 left1.setNeutralMode(NeutralMode.Coast);
		 
		 left2.follow(left1);
		 left2.setInverted(true);
		 left2.setNeutralMode(NeutralMode.Coast);
		 left3.follow(left1);
		 left3.setInverted(true);
		 left3.setNeutralMode(NeutralMode.Coast);
		 
		 right1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		 right1.setSensorPhase(false);
		 right1.setInverted(true);
		 right1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		 right1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		 right1.configNominalOutputForward(0, 0);
		 right1.configNominalOutputReverse(0, 0);
		 right1.configPeakOutputForward(1, 0);
		 right1.configPeakOutputReverse(-1, 0);
		 right1.selectProfileSlot(0, 0);
		 right1.config_kF(0, .5, 0);
		 right1.config_kP(0, 0, 0);
		 right1.configMotionCruiseVelocity(2500, 0);
		 right1.configMotionAcceleration(((int)(2500/3)), 0);
		 right1.setNeutralMode(NeutralMode.Coast);
		 
		 right2.follow(right2);
		 right2.setInverted(false);
		 right2.setNeutralMode(NeutralMode.Coast);
		 right3.follow(right3);
		 right3.setInverted(false);
		 right3.setNeutralMode(NeutralMode.Coast);
		 
	}
	
	public void setLeftRight(double left, double right){
		left1.set(ControlMode.PercentOutput, -left);
		left2.set(ControlMode.PercentOutput, -left);
		left3.set(ControlMode.PercentOutput, -left);
		
		right1.set(ControlMode.PercentOutput, -right);
		right2.set(ControlMode.PercentOutput, -right);
		right3.set(ControlMode.PercentOutput, -right);
		
		SmartDashboard.putNumber("Left", left);
		SmartDashboard.putNumber("Right", right);
	}
	
	public void driveTargert(double target){
		int counts = (int) (target*countsPerDistance);
		left1.set(ControlMode.MotionMagic, counts + encoderOffsetLeft);
		left2.follow(left1);
		left3.follow(left1);
		
		right1.set(ControlMode.MotionMagic, counts + encoderOffsetRight);
		right2.follow(right1);
		right3.follow(right1);
	}
	
	public void outputToSmartDashboard(){
		PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		
		pidgey.getGeneralStatus(genStatus);
		pidgey.getFusedHeading(fusionStatus);
		
		double currentAngle = fusionStatus.heading;
		SmartDashboard.putNumber("leftPos", left1.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("rightPos", right1.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("angle", currentAngle);
	}
	
	public void turnAngle(double angle, double speed){
		PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		
		double [] xyz_dps = new double [3];
		pidgey.getGeneralStatus(genStatus);
		pidgey.getRawGyro(xyz_dps);
		pidgey.getFusedHeading(fusionStatus);
		
		double currentAngle = fusionStatus.heading;
		double currentAngularRate = xyz_dps[2];
		
		double turnThrottle = (angle - currentAngle) * kpAngle;
		
		if (Math.abs(turnThrottle) > speed){
			turnThrottle = speed * Math.signum(turnThrottle);
		}
		
		setLeftRight(-turnThrottle, turnThrottle);
	}

	public boolean getOnPosition(double target){
		return (Math.abs(left1.getSelectedSensorPosition(0) - (target*countsPerDistance) + encoderOffsetLeft ) < (2*countsPerDistance));
	}
	
	public void setHighGear(boolean high){
		shifter.set(high);
	}
	
	public boolean getOnAngle(double target){
		PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		
		double [] xyz_dps = new double [3];
		pidgey.getGeneralStatus(genStatus);
		pidgey.getRawGyro(xyz_dps);
		pidgey.getFusedHeading(fusionStatus);
		
		double currentAngle = fusionStatus.heading;
		return (Math.abs(currentAngle - angleOffset - target) < 3) && (left1.getMotorOutputPercent() < .2);
	}
	
	public void resetEncoders(){
		encoderOffsetRight = right1.getSelectedSensorPosition(0);
		encoderOffsetLeft = left1.getSelectedSensorPosition(0);
	}
	
	public void resetGyro() {
		PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		
		double [] xyz_dps = new double [3];
		pidgey.getGeneralStatus(genStatus);
		pidgey.getRawGyro(xyz_dps);
		pidgey.getFusedHeading(fusionStatus);
		
		angleOffset = fusionStatus.heading;
	}
	
	public void resetSensors() {
		resetGyro();
		resetEncoders();
	}
}
