package org.usfirst.frc.team4392.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {

	private TalonSRX left1, left2, left3, right1, right2, right3;
	private PigeonIMU pidgey;
	
	private double kpAngle = 0.04;
	private double kdAngle = 0.004;
	
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
		 
		 left2.follow(left1);
		 left2.setInverted(true);
		 left3.follow(left1);
		 left3.setInverted(true);
		 
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
		 
		 right2.follow(right2);
		 right2.setInverted(false);
		 right3.follow(right3);
		 right3.setInverted(false);
		 
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
		left1.set(ControlMode.MotionMagic, target);
		left2.follow(left1);
		left3.follow(left1);
		
		right1.set(ControlMode.MotionMagic, target);
		right2.follow(right1);
		right3.follow(right1);
	}
	
	public void outputToSmartDashboard(){
		SmartDashboard.putNumber("leftSpeed", left1.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("rightSpeed", right1.getSelectedSensorPosition(0));
	}
	
	public void turnAngle(double angle){
		PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		
		double [] xyz_dps = new double [3];
		pidgey.getGeneralStatus(genStatus);
		pidgey.getRawGyro(xyz_dps);
		pidgey.getFusedHeading(fusionStatus);
		
		double currentAngle = fusionStatus.heading;
		double currentAngularRate = xyz_dps[2];
		
		double turnThrottle = (angle - currentAngle) * kpAngle - (currentAngularRate) * kdAngle;
		
		setLeftRight(-turnThrottle, turnThrottle);
		SmartDashboard.putNumber("TurnThrottle", turnThrottle);
	}

	public boolean getOnPosition(double target){
		return (Math.abs(left1.getSelectedSensorPosition(0) - target) < 3000);
	}
}
