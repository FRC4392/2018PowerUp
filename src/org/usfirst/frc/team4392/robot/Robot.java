/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4392.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import org.usfirst.frc.team4392.robot.subsystems.Drivetrain;
import org.usfirst.frc.team4392.robot.subsystems.Intake;
import org.usfirst.frc.team4392.robot.subsystems.Lift;
import org.usfirst.frc.team4392.util.CheesyDriveHelper;
import org.usfirst.frc.team4392.util.DriveSignal;


public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private XboxController controller = new XboxController(0);
	private XboxController controller2 = new XboxController(1);
	private Drivetrain drive = new Drivetrain();
	private Lift lift = new Lift();
	private Intake intake = new Intake();
	private Solenoid sol = new Solenoid(0);
	private Solenoid intakePivot = new Solenoid(1);
	private CheesyDriveHelper cheesyhelper = new CheesyDriveHelper();
	int state = 0;
	
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
	}

	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}
	
	@Override
	public void autonomousPeriodic() {
		
		switch(state){
		case 0:
			drive.driveTargert(40960);
			if (drive.getOnPosition(40960)) {
				state = 1;
			}
			break;
		case 1:
			drive.turnAngle(-90);
			break;
		default:
			break;
		}
	}


	@Override
	public void teleopPeriodic() {
		drive.outputToSmartDashboard();
		double left = controller.getY(Hand.kLeft);
		if (left < 0){
			left = left * left * -1;
		} else {
			left = left * left;
		}
		double right = controller.getX(Hand.kRight);
		if (right < 0){
			right = right * right * -1;
		} else {
			right = right * right;
		}
		boolean quick = controller.getBumper(Hand.kRight);
		DriveSignal signal = cheesyhelper.cheesyDrive(left, right, quick);
		
		
		drive.setLeftRight(signal.leftMotor, signal.rightMotor);
		
		//double position = controller2.getY(Hand.kLeft);
		//position = 2*4096*(position-1);
		//double position = -600;
		//if (controller2.getYButton()){
		//	position = -7106;
		//}
		
		double position = 0;
		if (controller2.getXButton()){
			position = 17*2;
		} else if (controller2.getYButton()){
			position = 44;
		}
		lift.setHeight(position);
		//lift.setPosition((int)position);
		
		sol.set(controller.getBumper(Hand.kLeft));
		
		intakePivot.set(!controller.getBumper(Hand.kLeft));
		if (controller2.getTriggerAxis(Hand.kRight) > .5){
			intake.setOpen();
		} else {
			intake.setClose();
		}
		
		if(controller2.getBumper(Hand.kRight)){
			
			intake.setIntake();
		} else if (controller2.getBumper(Hand.kLeft)){
			intake.setOuttake();
		} else {
			intake.stop();
		}
	}


	@Override
	public void testPeriodic() {
	}
	
	
}
