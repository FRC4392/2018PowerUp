/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4392.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4392.robot.subsystems.Drivetrain;
import org.usfirst.frc.team4392.robot.subsystems.Intake;
import org.usfirst.frc.team4392.robot.subsystems.Lift;
import org.usfirst.frc.team4392.util.CheesyDriveHelper;
import org.usfirst.frc.team4392.util.DriveSignal;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;


public class Robot extends IterativeRobot {
	private XboxController controller = new XboxController(0);
	private XboxController controller2 = new XboxController(1);
	private Drivetrain drive = new Drivetrain();
	private Lift lift = new Lift();
	private Intake intake = new Intake();
	
	private CheesyDriveHelper cheesyhelper = new CheesyDriveHelper();
	private int autoStartPostion = 0;
	private SendableChooser autoStartChooser;
	private String autoPlatePosition = "";
	private int selectedAuto = 0; 
		//0 = drive straight
		//1 = switch from left
		//2 = scale from left
		//3 = left switch from middle
		//4 = right switch from middle
		//5 = scale from right
		//6 = switch from right
	private Timer autoTimer = new Timer();
	
	
	int state = 0;
	boolean pivotState = false;
	boolean lastPivotButtonState = false;
	
	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture(); 
		autoStartChooser = new SendableChooser();
		autoStartChooser.addDefault("Left", 0);
		autoStartChooser.addObject("Center", 1);
		autoStartChooser.addObject("Right", 2);
		autoStartChooser.addObject("Drive Straight", 3);
		SmartDashboard.putData(autoStartChooser);
		
	}

	@Override
	public void autonomousInit() {
		autoStartPostion = (int) autoStartChooser.getSelected();
		autoPlatePosition = DriverStation.getInstance().getGameSpecificMessage();
		
		switch(autoStartPostion){
		case 0: //start left
			if(autoPlatePosition.charAt(0) == 'L'){
				//go to switch
				selectedAuto = 1;
			} else if(autoPlatePosition.charAt(1) == 'L'){
				selectedAuto = 2;
			} else {
				selectedAuto = 0;
			}
			break;
		case 1: //start middle
			if(autoPlatePosition.charAt(0) == 'L'){
				//go to switch
				selectedAuto = 3;
			} else if(autoPlatePosition.charAt(0) == 'R'){
				selectedAuto = 4;
			} else {
				selectedAuto = 0;
			}
			break;
		case 2: //start right
			if(autoPlatePosition.charAt(0) == 'R'){
				//go to switch
				selectedAuto = 6;
			} else if(autoPlatePosition.charAt(1) == 'R'){
				selectedAuto = 5;
			} else {
				selectedAuto = 0;
			}
			break;
		case 3: //drive straight
		default:
			selectedAuto = 0;
			break;
		}
		autoTimer.start();
	}
	
	@Override
	public void autonomousPeriodic() {
		switch(selectedAuto){
		case 0:
			driveStraight();
			break;
		case 1:
			Auto1();
			break;
		case 2:
			Auto2();
			break;
		case 3:
			Auto3();
			break;
		case 4:
			Auto4();
			break;
		case 5:
			Auto5();
			break;
		case 6:
			Auto6();
			break;
		default:
			driveStraight();
		}
	}


	@Override
	public void teleopPeriodic() {
		drive.outputToSmartDashboard();
		
		//Drivetrain controls
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
		drive.setHighGear(controller.getBumper(Hand.kLeft));
		
		if (controller2.getXButton()){
			lift.setPosition(17*2);
		} else if (controller2.getYButton()){
			lift.setPosition(55);
		} else if (controller2.getBButton()){
			lift.setPosition(70);
		} else if (controller2.getAButton()){
			lift.setPosition(1);
		} else if (controller2.getBumper(Hand.kRight)){
			lift.setPosition(0);
		}
		
		//Intake Controls
		if ((controller2.getTriggerAxis(Hand.kLeft) > .5) && !lastPivotButtonState){
			pivotState = !pivotState;
		}
		lastPivotButtonState = controller2.getTriggerAxis(Hand.kLeft) > .5;
		intake.setPivot(pivotState);
		
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
	
	//0 = drive straight
	//1 = switch from left
	//2 = scale from left
	//3 = left switch from middle
	//4 = right switch from middle
	//5 = scale from right
	//6 = switch from right
	public void driveStraight(){
		drive.driveTargert(120);
	}
	
	public void Auto1(){ 	//1 = switch from left
		switch(state){
		case 0:
			drive.driveTargert(168);
			if(drive.getOnPosition(168)){
				state++;
			}
			break;
		case 1:
			intake.setPivotDown();
			lift.setHeight(17*2);
			if(lift.getOnHeight(17*2)){
				state++;
			}
			break;
		case 2:
			drive.turnAngle(-90, .5);
			if(drive.getOnAngle(-90)){
				state++;
			}
			break;
		case 3:
			drive.resetEncoders();
			state++;
			break;
		case 4:
			drive.driveTargert(25.3);
			if(drive.getOnPosition(25.3)){
				state++;
			}
			break;
		case 5:
			autoTimer.reset();
			break;
		case 6:
			intake.setOuttake();
			if (autoTimer.get() > 2){
				state++;
			}
			break;
		case 7:
			intake.stop();
			drive.resetEncoders();
			state++;
			break;
		case 8:
			drive.driveTargert(-20);
			if (drive.getOnPosition(-20)){
				state++;
			}
		case 9:
			lift.setHeight(0);
		}
	}
	
	public void Auto2(){	//2 = scale from left
		/*switch(state){
		case 0:
			drive.driveTargert(10);
			if(drive.getOnPosition(10)){
				state++;
			}
			break;
		case 1:
			intake.setPivotDown();
			lift.setHeight(17*2);
			if(lift.getOnHeight(17*2)){
				state++;
			}
			break;
		case 2:
			drive.turnAngle(-90);
			if(drive.getOnAngle(-90)){
				state++;
			}
			break;
		case 3:
			drive.resetEncoders();
			state++;
			break;
		case 4:
			drive.driveTargert(10);
			if(drive.getOnPosition(10)){
				state++;
			}
			break;
		case 5:
			autoTimer.reset();
			break;
		case 6:
			intake.setOuttake();
			if (autoTimer.get() > 2){
				state++;
			}
			break;
		case 7:
			intake.stop();
			drive.resetEncoders();
			state++;
			break;
		case 8:
			drive.driveTargert(-10);
			if (drive.getOnPosition(-10)){
				state++;
			}
		case 9:
			lift.setHeight(0);
		}*/
		driveStraight();
	}
	
	public void Auto3(){	//3 = left switch from middle
		switch(state){
		case 0:
			drive.driveTargert(36);
			if(drive.getOnPosition(10)){
				state++;
			}
			break;
		case 1:
			drive.turnAngle(-90, .5);
			if(drive.getOnAngle(-90)){
				state++;
			}
			break;
		case 2:
			drive.resetEncoders();
			state++;
			break;
		case 3:
			drive.driveTargert(127.75);
			if(drive.getOnPosition(127.75)){
				state++;
			}
			break;
		case 4:
			drive.turnAngle(0, 0.5);
			if(drive.getOnAngle(0)){
				state++;
			}
		case 5:
			intake.setPivotDown();
			lift.setHeight(17*2);
			if(lift.getOnHeight(17*2)){
				state++;
			}
			break;
		case 6:
			drive.resetEncoders();
			state++;
			break;
		case 7:
			drive.driveTargert(104);
			if(drive.getOnPosition(104)){
				state++;
			}
			break;
		case 8:
			autoTimer.reset();
			state++;
			break;
		case 9:
			intake.setOuttake();
			if (autoTimer.get() > 2){
				state++;
			}
			break;
		case 10:
			drive.resetEncoders();
			state++;
			break;
		case 11:
			drive.driveTargert(-20);
			if(drive.getOnPosition(-20)){
				state++;
			}
			break;
		case 12:
			lift.setHeight(0);
		}
	}
	
	public void Auto4(){	//4 = right switch from middle
		switch(state){
		case 0:
			drive.driveTargert(36);
			if(drive.getOnPosition(10)){
				state++;
			}
			break;
		case 1:
			drive.turnAngle(90, .5);
			if(drive.getOnAngle(90)){
				state++;
			}
			break;
		case 2:
			drive.resetEncoders();
			state++;
			break;
		case 3:
			drive.driveTargert(125.75);
			if(drive.getOnPosition(125.75)){
				state++;
			}
			break;
		case 4:
			drive.turnAngle(0, 0.5);
			if(drive.getOnAngle(0)){
				state++;
			}
		case 5:
			intake.setPivotDown();
			lift.setHeight(17*2);
			if(lift.getOnHeight(17*2)){
				state++;
			}
			break;
		case 6:
			drive.resetEncoders();
			state++;
			break;
		case 7:
			drive.driveTargert(104);
			if(drive.getOnPosition(104)){
				state++;
			}
			break;
		case 8:
			autoTimer.reset();
			state++;
			break;
		case 9:
			intake.setOuttake();
			if (autoTimer.get() > 2){
				state++;
			}
			break;
		case 10:
			drive.resetEncoders();
			state++;
			break;
		case 11:
			drive.driveTargert(-20);
			if(drive.getOnPosition(-20)){
				state++;
			}
			break;
		case 12:
			lift.setHeight(0);
		}
	}
	
	public void Auto5(){	//5 = scale from right
		/*switch(state){
		case 0:
			drive.driveTargert(10);
			if(drive.getOnPosition(10)){
				state++;
			}
			break;
		case 1:
			intake.setPivotDown();
			lift.setHeight(17*2);
			if(lift.getOnHeight(17*2)){
				state++;
			}
			break;
		case 2:
			drive.turnAngle(-90);
			if(drive.getOnAngle(-90)){
				state++;
			}
			break;
		case 3:
			drive.resetEncoders();
			state++;
			break;
		case 4:
			drive.driveTargert(10);
			if(drive.getOnPosition(10)){
				state++;
			}
			break;
		case 5:
			autoTimer.reset();
			break;
		case 6:
			intake.setOuttake();
			if (autoTimer.get() > 2){
				state++;
			}
			break;
		case 7:
			intake.stop();
			drive.resetEncoders();
			state++;
			break;
		case 8:
			drive.driveTargert(-10);
			if (drive.getOnPosition(-10)){
				state++;
			}
		case 9:
			lift.setHeight(0);
		}*/
		driveStraight();
	}
	
	public void Auto6(){	//6 = switch from right
		switch(state){
		case 0:
			drive.driveTargert(168);
			if(drive.getOnPosition(168)){
				state++;
			}
			break;
		case 1:
			intake.setPivotDown();
			lift.setHeight(17*2);
			if(lift.getOnHeight(17*2)){
				state++;
			}
			break;
		case 2:
			drive.turnAngle(90, .5);
			if(drive.getOnAngle(90)){
				state++;
			}
			break;
		case 3:
			drive.resetEncoders();
			state++;
			break;
		case 4:
			drive.driveTargert(25.3);
			if(drive.getOnPosition(25.3)){
				state++;
			}
			break;
		case 5:
			autoTimer.reset();
			break;
		case 6:
			intake.setOuttake();
			if (autoTimer.get() > 2){
				state++;
			}
			break;
		case 7:
			intake.stop();
			drive.resetEncoders();
			state++;
			break;
		case 8:
			drive.driveTargert(-20);
			if (drive.getOnPosition(-20)){
				state++;
			}
		case 9:
			lift.setHeight(0);
		}
	}
	
	
	
}