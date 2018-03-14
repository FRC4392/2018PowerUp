package org.usfirst.frc.team4392.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Lift {
	private TalonSRX lift1 = new TalonSRX(4);
	private TalonSRX lift2 = new TalonSRX(7);
	
	private static double offset = -400;
	private static double diameter = 1.273;
	private static double circumference = Math.PI * diameter;
	private static double countsPerRot = 4096;
	private static double distancePerCount = circumference/countsPerRot;
	private static double countsPerDistance = countsPerRot/circumference;
	
	private static double setPosition = 0;
	
	public Lift(){
		lift1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		lift1.setSensorPhase(false);
		lift1.setInverted(false);
		lift1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		lift1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		lift1.configNominalOutputForward(0, 0);
		lift1.configNominalOutputReverse(0, 0);
		lift1.configPeakOutputForward(1, 0);
		lift1.configPeakOutputReverse(-1, 0);
		lift1.selectProfileSlot(0, 0);
		lift1.config_kF(0, .16, 0);
		lift1.config_kP(0, .8, 0);
		lift1.configMotionCruiseVelocity(6387, 0);
		lift1.configMotionAcceleration(((int)(6387*2)), 0);
		lift1.setNeutralMode(NeutralMode.Brake);
		
		lift2.setInverted(true);
		lift2.follow(lift1);
		lift2.setNeutralMode(NeutralMode.Brake);
	}
	
	public void setPosition(int position){
		lift1.set(ControlMode.MotionMagic, position);
		lift2.follow(lift1);
	}
	
	public void setPower(double Power) {
		lift1.set(ControlMode.PercentOutput, Power);
		lift2.follow(lift1);
	}
	
	public void setHeight(double inches){
		double counts = (countsPerDistance * inches * -1 + offset)/2;
		setPosition((int)counts);
	}
	
	public double getHeight() {
		return lift1.getSelectedSensorPosition(0) * distancePerCount;
	}
	
	public boolean getOnHeight(double target){
		return (Math.abs(getHeight() - target) < 3);
	}
	
}
