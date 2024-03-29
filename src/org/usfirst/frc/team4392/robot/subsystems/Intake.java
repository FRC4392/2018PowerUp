package org.usfirst.frc.team4392.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {
	
	double intakeSpeed = -1;
	double outtakeSpeed = .5;

	//Talon 1 declaration
	//Talon 2 declaration
	private TalonSRX intake1 = new TalonSRX(5);
	private TalonSRX intake2 = new TalonSRX(6);
	private Solenoid jaw = new Solenoid(2);
	private Solenoid Pivot = new Solenoid(1);
	
	//
	public Intake(){
		intake1.follow(intake2);
		intake1.setInverted(true);
		intake1.setNeutralMode(NeutralMode.Brake);
		intake2.setNeutralMode(NeutralMode.Brake);
	}
	
	public void setSpeed(double speed){
		intake2.set(ControlMode.PercentOutput, speed);
	}
	
	public void setIntake(){
		setSpeed(intakeSpeed);
	}
	
	public void setOuttake(){
		setSpeed(outtakeSpeed);
	}
	
	public void stop(){
		setSpeed(0);
	}
	
	public void setOpen(){
		jaw.set(true);
	}
	
	public void setClose(){
		jaw.set(false);
	}
	
	public void setPivotUp(){
		setPivot(true);
	}
	
	public void setPivotDown(){
		setPivot(false);
	}
	
	public void setPivot(boolean down){
		Pivot.set(down);
	}
}
