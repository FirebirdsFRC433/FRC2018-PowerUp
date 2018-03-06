/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team433.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	// Drive train
	MecanumDrive myRobot;
	WPI_TalonSRX leftFront = new WPI_TalonSRX(5);
	WPI_TalonSRX leftBack = new WPI_TalonSRX(4);
	WPI_TalonSRX rightFront = new WPI_TalonSRX(3);
	WPI_TalonSRX rightBack = new WPI_TalonSRX(2);

	double straight_encRevsLF;
	// double straight_encRevsLB;
	double straight_encRevsRF;
	double straight_encRevsRB;

	double strafe_encRevsLF;
	// double strafe_encRevsLB;
	double strafe_encRevsRF;
	double strafe_encRevsRB;

	Joystick joystick = new Joystick(0);
	Joystick xbox = new Joystick(1);

	// Elevator
	WPI_TalonSRX elevatorExtension1 = new WPI_TalonSRX(6);
	WPI_TalonSRX elevatorExtension2 = new WPI_TalonSRX(7);
	double encRevsElevator; // encoder for elevator
	DigitalInput elevSwitchHi = new DigitalInput(0);
	DigitalInput elevSwitchLo = new DigitalInput(1);

	// cube intake & output
	VictorSPX intakeMotor1 = new VictorSPX(0);
	VictorSPX intakeMotor2 = new VictorSPX(1);

	// Hanger
	WPI_TalonSRX hangMotor1 = new WPI_TalonSRX(8);
	WPI_TalonSRX hangMotor2 = new WPI_TalonSRX(9);

	// Sensors
	AnalogInput ultrasonic = null;
	AHRS navx;

	// Autonomous Switches
	DigitalInput autonSwitchBlue = new DigitalInput(2);
	DigitalInput autonSwitchRed = new DigitalInput(4);
	DigitalInput autonSwitchGreen = new DigitalInput(5);
	DigitalInput autonSwitchYellow = new DigitalInput(6);
	int switchGreenFinal;
	int switchBlueFinal;
	int switchRedFinal;
	int switchYellowFinal;
	int switBinFin;

	String gameData;

	// autonomous constants
	double defaultSpeed;
	double distScale;
	double distPlatZone;
	double distSwitch;
	double distCross;
	double distPlatToScale;
	double releaseCube;
	int stepNumber;
	double timerDelay;
	int whereIsIt;

	CameraServer server;

	@Override
	public void robotInit() {
		myRobot = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
		navx = new AHRS(SerialPort.Port.kUSB);

		leftFront.setSensorPhase(true);
		leftBack.setSensorPhase(true);
		rightFront.setSensorPhase(true);
		rightBack.setSensorPhase(true);

		leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		leftBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rightBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		// setting up the elevator as a PID device
		elevatorExtension1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		elevatorExtension1.configNominalOutputReverse(-.8, 10);
		elevatorExtension1.configNominalOutputForward(.8, 10);
		elevatorExtension1.setSensorPhase(true);
		elevatorExtension1.selectProfileSlot(0, 0);
		elevatorExtension1.config_kF(0, .85, 10);
		elevatorExtension1.config_kP(0, .7, 10);
		elevatorExtension1.config_kI(0, 0, 10); // .1
		elevatorExtension1.config_kD(0, 0, 10); // .3
		elevatorExtension1.configMotionCruiseVelocity(2000, 10);
		elevatorExtension1.configMotionAcceleration(4400, 10);

		elevatorExtension2.set(ControlMode.Follower, elevatorExtension1.getDeviceID());
	}

	@Override
	public void autonomousInit() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		defaultSpeed = 0.5;
		releaseCube = -0.6;
		distScale = 285; // From drive station wall to scale's null territory
		distPlatZone = 227; // From drive station wall to in between switch and
							// platforms
		distSwitch = 130; // From drive station wall to horizontal side of scale
							// (side perpendicular to
							// drive station)
		distCross = 200; // From one side of switch to other
		distPlatToScale = 97; // To get from distPlatZone to scale's null
								// territory
		stepNumber = 1;
		navx.zeroYaw();

		timerDelay = .1;
	}

	@Override
	public void autonomousPeriodic() {

		// divided by 259 to convert encoder loops to inches
		straight_encRevsLF = leftFront.getSelectedSensorPosition(0) / 209.55;
		// straight_encRevsLB = leftBack.getSelectedSensorPosition(0) / 229.34;
		straight_encRevsRF = -rightFront.getSelectedSensorPosition(0) / 207.94;
		straight_encRevsRB = -rightBack.getSelectedSensorPosition(0) / 205.51;

		strafe_encRevsLF = leftFront.getSelectedSensorPosition(0) / 129.77;
		// strafe_encRevsLB = leftBack.getSelectedSensorPosition(0);
		strafe_encRevsRF = -rightFront.getSelectedSensorPosition(0) / 119.51;
		strafe_encRevsRB = -rightBack.getSelectedSensorPosition(0) / 139.13;

		encRevsElevator = elevatorExtension1.getSelectedSensorPosition(0);

		// switch code
		boolean switRawGreen = autonSwitchGreen.get();
		boolean switRawBlue = autonSwitchBlue.get();
		boolean switRawRed = autonSwitchRed.get();
		boolean switRawYellow = autonSwitchYellow.get();
		SmartDashboard.putBoolean("Green", switRawGreen);
		SmartDashboard.putBoolean("Blue", switRawBlue);
		SmartDashboard.putBoolean("Red", switRawRed);
		SmartDashboard.putBoolean("Yellow", switRawYellow);

		if (switRawGreen) {
			switchGreenFinal = 1;
		} else {
			switchGreenFinal = 0;
		}

		if (switRawBlue) {
			switchBlueFinal = 1;
		} else {
			switchBlueFinal = 0;
		}

		if (switRawRed) {
			switchRedFinal = 1;
		} else {
			switchRedFinal = 0;
		}
		if (switRawYellow) {
			switchYellowFinal = 1;
		} else {
			switchYellowFinal = 0;
		}

		int switBinFin = (switchYellowFinal * 8) + (switchGreenFinal * 4) + (switchBlueFinal * 2) + switchRedFinal;
		SmartDashboard.putNumber("SwitBinFin", switBinFin);

		switch (switBinFin) {
		case 0:
			DoNothing(); // no switches on
			break;
		case 1:
			LeftScale(); // red on blue off green off
			break;
		case 2:
			CenterSwitch(); // blue on red off green off
			break;
		case 3:
			LeftSwitch(); // blue on red on green off
			break;
		case 4:
			RightScale(); // green on red off blue off
			break;
		case 5:
			RightSwitch(); // green on blue off red on
			break;
		case 6:
			LeftGen(); // green on blue on red off
			break;
		case 7:
			CrossLine(); // all on
			break;
		case 8:
			RightGen(); // yellow on break;
		}
	}

	public void DoNothing() { // do nothing
		myRobot.driveCartesian(0, 0, 0);
	}

	public void LeftScale() { // starting configuration left side touching
								// driver wall
		if (gameData.charAt(1) == 'R') { // going for right side of scale
			if (stepNumber == 1) {
				if (straight_encRevsRF > -distPlatZone) { // straight right to
															// platform zone
															// (between switch and
															// scale)
					myRobot.driveCartesian(defaultSpeed, 0, 0);
				} else if (straight_encRevsRF <= -distPlatZone) {
					stepNumber = 2;
					leftFront.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsLF > -distCross) { // cross field to other
														// side of switch
					leftFront.getSelectedSensorPosition(0);
					myRobot.driveCartesian(0, -defaultSpeed, 0);
				} else if (straight_encRevsLF <= -distCross) {
					stepNumber = 3;
					rightBack.setSelectedSensorPosition(0, 0, 10);
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 3) {
				if (straight_encRevsRF < distPlatToScale) { // drive to
															// scale
					rightBack.getSelectedSensorPosition(0);
					myRobot.driveCartesian(defaultSpeed, 0, 0);
				} else if (straight_encRevsRB >= distPlatToScale) {
					myRobot.driveCartesian(0, 0, 0);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
		} else if (gameData.charAt(1) == 'L') {
			if (stepNumber == 1) { // jerk motion
				if (straight_encRevsRF < 10) { // go straight to scale
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(timerDelay);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= distScale) { // continue going straight to scale
					myRobot.driveCartesian(0, defaultSpeed, 0);
				} else if (straight_encRevsRF > distScale) {
					stepNumber = 3;
					leftFront.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 3) {
				if (straight_encRevsLF < 45) { // turn towards scale
					myRobot.driveCartesian(0, 0, .4);
				} else if (straight_encRevsLF >= 45) {
					rightBack.setSelectedSensorPosition(0, 0, 10);
					stepNumber = 4;
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 4) { // back away from scale
				/*
				 * if (elevSwitchHi.get()) { elevatorExtension1.set(0); } else {
				 * elevatorExtension1.set(ControlMode.MotionMagic, 60000);
				 */

			}
			if (straight_encRevsRB > -15) {
				rightBack.getSelectedSensorPosition(0);
				myRobot.driveCartesian(0, -defaultSpeed, 0);
			} else if (Math.abs(encRevsElevator) > 59000) {
				stepNumber = 5;
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
			/*
			 * } else if (stepNumber == 5) { intakeMotor1.set(ControlMode.PercentOutput,
			 * -releaseCube); intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			 */

		} else {
			myRobot.driveCartesian(0, 0, 0);
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
	}

	public void CenterSwitch() { // starting configuration back bumper touching
									// wall and left bumper aligned with
									// exchange zone
		if (gameData.charAt(0) == 'L') { // jerk code
			SmartDashboard.putNumber("Step Number", stepNumber);
			if (stepNumber == 1) {
				if (straight_encRevsRB < 10) {
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRB >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(.3);
					stepNumber = 2;
				} else {
				}
			}
			if (stepNumber == 2) {
				if (straight_encRevsRB < 24) { // move forward past exchange zone ramp
					myRobot.driveCartesian(0, .5, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRB >= 24) {
					leftFront.getSelectedSensorPosition(0);
					stepNumber = 3;
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 3) {
				if (strafe_encRevsLF > -105) {
					myRobot.driveCartesian(-.4, 0, -.03);
					rightFront.setSelectedSensorPosition(0, 0, 10);
				} else if (strafe_encRevsLF <= -105 && straight_encRevsRF < 63) {
					stepNumber = 4;
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 4) {
				if (straight_encRevsRF < 63) { // move forward to switch
					rightFront.getSelectedSensorPosition(0);
					myRobot.driveCartesian(0, .4, 0);
				} else if (straight_encRevsRF >= 63 && encRevsElevator > 27000) { // place cube
					intakeMotor1.set(ControlMode.PercentOutput, -releaseCube);
					intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else {
			}
			elevatorExtension1.set(ControlMode.MotionMagic, 28000); // extend elevator
		}

		else if (gameData.charAt(0) == 'R') {

			SmartDashboard.putNumber("Left Front Strafe Auton", strafe_encRevsLF);
			SmartDashboard.putNumber("Step Number Auton", stepNumber);
			SmartDashboard.putNumber("Left Front Encoder Auton", straight_encRevsLF);
			SmartDashboard.putNumber("Right Front Encoder Auton", straight_encRevsRF);
			SmartDashboard.putNumber("Right Back Encoder Auton", straight_encRevsRB);
			SmartDashboard.putNumber("Elevator Encoder Auton", encRevsElevator);
			SmartDashboard.putNumber("Where is it now?", whereIsIt);

			// UNTESTED - TWO CUBE
			/*if (stepNumber < 150) { // first cube
				if (strafe_encRevsLF < 105) { // move until past pyramid
					myRobot.driveCartesian(defaultSpeed, 0, .03);
					rightFront.setSelectedSensorPosition(0, 0, 10);
				} else if (strafe_encRevsLF >= 105 && straight_encRevsRF < 110) { // forward to switch
					rightFront.getSelectedSensorPosition(0);
					myRobot.driveCartesian(0, defaultSpeed, 0);
				} else if (straight_encRevsRF >= 110 && encRevsElevator > 27000) { // place cube
					intakeMotor1.set(ControlMode.PercentOutput, -defaultSpeed);
					intakeMotor2.set(ControlMode.PercentOutput, defaultSpeed);
					stepNumber++;
					rightBack.setSelectedSensorPosition(0, 0, 10);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
				elevatorExtension1.set(ControlMode.MotionMagic, 28000);
			} else if (stepNumber >= 150) { // second cube
				if (straight_encRevsRB > -20) { // turn towards pyramid
					myRobot.driveCartesian(0, 0, -.4);
					leftFront.setSelectedSensorPosition(0, 0, 10);
					rightFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRB <= -20 && encRevsElevator < 200) {// Move on to next step once turn is
																				// done and elevator has moved down
																				// enough
					stepNumber = 151;
					leftFront.getSelectedSensorPosition(0);
					rightFront.getSelectedSensorPosition(0);
				}
				elevatorExtension1.set(ControlMode.MotionMagic, 100);
			} else if (stepNumber >= 151 && stepNumber < 200) {
				if (straight_encRevsLF < 30) { // move towards pyramid
					myRobot.driveCartesian(0, defaultSpeed, 0);
					intakeMotor1.set(ControlMode.PercentOutput, defaultSpeed);
					intakeMotor2.set(ControlMode.PercentOutput, -defaultSpeed);
				} else if (straight_encRevsLF >= 30) { // once at pyramid, keep intake going to pick up cube
					intakeMotor1.set(ControlMode.PercentOutput, defaultSpeed);
					intakeMotor2.set(ControlMode.PercentOutput, -defaultSpeed);
					stepNumber++;
				}
			} else if (stepNumber == 200) {
				if (straight_encRevsRF > 20) {// moving backwards with cube
					myRobot.driveCartesian(0, -defaultSpeed, 0);
					rightBack.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRF < 2) { // once moved back enough, go to next step
					stepNumber = 201;
					rightBack.getSelectedSensorPosition(0);
				}
			} else if (stepNumber == 201) {
				elevatorExtension1.set(ControlMode.MotionMagic, 28000); // raising the elevator with cube, parallel to
																		// switch
				if (encRevsElevator > 27000) { // once elevator has gotten high enough, move on to next step
					stepNumber = 202;
				} else {
				}
			} else if (stepNumber == 202) {
				if (straight_encRevsRB < 20) { // turn right towards switch
					myRobot.driveCartesian(0, 0, -.4);
					leftFront.setSelectedSensorPosition(0, 0, 10);
					rightFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRB >= 20) {// once done turning, go to next step
					stepNumber = 203;
					leftFront.getSelectedSensorPosition(0);
					rightFront.getSelectedSensorPosition(0);
				}
			} else if (stepNumber == 203) { // now facing switch. Release cube
				intakeMotor1.set(ControlMode.PercentOutput, -defaultSpeed);
				intakeMotor2.set(ControlMode.PercentOutput, defaultSpeed);
			} else {
			} */

			// TESTED AND WORKS - ONE CUBE

			if (strafe_encRevsLF < 115) { // move until past pyramid
				myRobot.driveCartesian(defaultSpeed, 0, .03);
				rightFront.setSelectedSensorPosition(0, 0, 10);
				whereIsIt = 1;
			} else if (strafe_encRevsLF >= 115 && straight_encRevsRF < 110) { // forward to switch
				rightFront.getSelectedSensorPosition(0);
				myRobot.driveCartesian(0, defaultSpeed, 0);
				whereIsIt = 2;
			} else if (straight_encRevsRF >= 110 && encRevsElevator > 27000) { // place cube
				intakeMotor1.set(ControlMode.PercentOutput, defaultSpeed);
				intakeMotor2.set(ControlMode.PercentOutput, defaultSpeed);
				whereIsIt = 3;
			} else {
				myRobot.driveCartesian(0, 0, 0);
				whereIsIt = 4;
			}
			elevatorExtension1.set(ControlMode.MotionMagic, 28000); 
			


		} else {
			myRobot.driveCartesian(0, 0, 0);
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
	}

	public void LeftSwitch() { // starting configuration is backwards
		if (gameData.charAt(0) == 'R') {
			if (straight_encRevsLF >= -distPlatZone) { // backwards past switch
				myRobot.driveCartesian(0, -defaultSpeed, 0);
				rightFront.setSelectedSensorPosition(0, 0, 10);
			} else if (straight_encRevsRF < distCross && straight_encRevsLF < -distPlatZone) { // across
																								// to
																								// scale
																								// platform
				rightFront.getSelectedSensorPosition(0);
				myRobot.driveCartesian(-defaultSpeed, 0, 0);
			} else if (straight_encRevsRF >= distCross) { // place cube
				myRobot.driveCartesian(0, 0, 0);
				stepNumber = 2;
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
			elevatorExtension1.set(ControlMode.MotionMagic, 28000);
		}

		else if (gameData.charAt(0) == 'L') {
			if (stepNumber == 1) { // jerk motion
				if (straight_encRevsRF < 10) { // go straight to switch
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(timerDelay);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= distSwitch) { // continue going straight to switch
					myRobot.driveCartesian(0, defaultSpeed, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRF > distSwitch) {
					stepNumber = 3;
					leftFront.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 3) {
				if (straight_encRevsLF < 45) { // turn towards switch
					myRobot.driveCartesian(0, 0, .4);
				} else if (straight_encRevsLF >= 45) {
					stepNumber = 4;
				}
			} else if (stepNumber == 4) {
				myRobot.driveCartesian(0, 0, 0);
				if (encRevsElevator > 27000) {
					intakeMotor1.set(ControlMode.PercentOutput, -releaseCube);
					intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
			/*
			 * if (elevSwitchHi.get()) { elevatorExtension1.set(0); } else {
			 * elevatorExtension1.set(ControlMode.MotionMagic, 28000); }
			 */
		} else {
			myRobot.driveCartesian(0, 0, 0);
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
	}

	public void RightScale() { // starting configuration is back side on wall
		if (gameData.charAt(1) == 'R') {
			if (stepNumber == 1) { // jerk code
				if (straight_encRevsRF < 10) { // go straight toward scale
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(timerDelay);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF < 285) { // continue going straight toward scale
					myRobot.driveCartesian(0, defaultSpeed, -.01);
				} else if (straight_encRevsRF >= 285) {
					stepNumber = 3;
					rightBack.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 3) { // turn toward scale
				SmartDashboard.putNumber("Right Rear Encoder", rightBack.getSelectedSensorPosition(0));
				SmartDashboard.putNumber("Step Number", stepNumber);
				if (Math.abs(straight_encRevsRB) < 45) {
					myRobot.driveCartesian(0, 0, -defaultSpeed);
				} else if (Math.abs(straight_encRevsRB) >= 45) {
					myRobot.driveCartesian(0, 0, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
					stepNumber = 4;
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 4) { // back away from scale
				/*
				 * if (elevSwitchHi.get()) { elevatorExtension1.set(0); } else {
				 * elevatorExtension1.set(ControlMode.MotionMagic, 60000);
				 */
				if (straight_encRevsLF > -15) {
					leftFront.getSelectedSensorPosition(0);
					myRobot.driveCartesian(0, -defaultSpeed, 0);
				} else if (Math.abs(encRevsElevator) > 59000) {
					stepNumber = 5;
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 5) {
				// intakeMotor1.set(ControlMode.PercentOutput, -releaseCube);
				// intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}

		} else if (gameData.charAt(1) == 'L') {
			if (stepNumber == 1) {
				if (strafe_encRevsLF > -distPlatZone) { // strafe left past
														// switch
					myRobot.driveCartesian(-defaultSpeed, 0, 0);
					rightFront.setSelectedSensorPosition(0, 0, 10);
				} else if (strafe_encRevsLF <= -distPlatZone && straight_encRevsRF > -distCross) { // backwards
																									// next
																									// to
																									// scale
					rightFront.getSelectedSensorPosition(0);
					myRobot.driveCartesian(0, -defaultSpeed, 0);
					rightBack.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRF <= -distCross && strafe_encRevsRB > -distPlatToScale) {
					stepNumber = 2;
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= -distCross && strafe_encRevsRB > -distPlatToScale) { // strafe
																								// left
																								// to
																								// line
																								// up
																								// with
																								// scale
					rightBack.getSelectedSensorPosition(0);
					myRobot.driveCartesian(-defaultSpeed, 0, 0);
				} else if (strafe_encRevsRB <= -distPlatToScale) { // place cube
					intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
					intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
		} else {
			myRobot.driveCartesian(0, 0, 0);
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
	}

	public void RightSwitch() { // starting configuration is backwards
		if (gameData.charAt(0) == 'L') {
			if (straight_encRevsLF >= -distPlatZone) {
				myRobot.driveCartesian(0, -defaultSpeed, 0);
				rightFront.setSelectedSensorPosition(0, 0, 10);
			} else if (straight_encRevsRF <= 285 && straight_encRevsLF < -distPlatZone) {
				rightFront.getSelectedSensorPosition(0);
				myRobot.driveCartesian(defaultSpeed, 0, 0);
			} else if (straight_encRevsRF > 285 && straight_encRevsLF < -distPlatZone) { // place
																							// cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
			elevatorExtension1.set(ControlMode.MotionMagic, 28000);
		}

		else if (gameData.charAt(0) == 'R') {
			if (stepNumber == 1) { // jerk motion
				if (straight_encRevsRF < 10) { // go straight to switch
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(.1);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= distSwitch) { // continue going straight to switch
					myRobot.driveCartesian(0, defaultSpeed, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRF > distSwitch) {
					stepNumber = 3;
					leftFront.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 3) {
				if (straight_encRevsLF > -20) { // turn towards switch
					myRobot.driveCartesian(0, 0, -.4);
				} else if (straight_encRevsLF <= -20) {
					stepNumber = 4;
				}
			} else if (stepNumber == 4) {
				myRobot.driveCartesian(0, 0, 0);
				if (encRevsElevator > 27000) {
					// intakeMotor1.set(ControlMode.PercentOutput, -releaseCube);
					// intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
			/*
			 * if (elevSwitchHi.get()) { elevatorExtension1.set(0); } else {
			 * elevatorExtension1.set(ControlMode.MotionMagic, 28000); }
			 */

		} else {
			myRobot.driveCartesian(0, 0, 0);
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
	}

	public void LeftGen() { // starting configuration: back bumper on wall
		SmartDashboard.putNumber("Right Front Encoder", straight_encRevsRF);
		SmartDashboard.putNumber("Right Back Encoder", straight_encRevsRB);
		SmartDashboard.putNumber("Left Front Encoder", straight_encRevsLF);
		SmartDashboard.putNumber("Step Number", stepNumber);
		SmartDashboard.putNumber("Program", switBinFin);

		if (gameData.charAt(0) == 'L' && gameData.charAt(1) != 'L') { // going for switch
			if (stepNumber == 1) { // jerk motion
				if (straight_encRevsRF < 10) { // go straight to switch
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(timerDelay);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= distSwitch) { // continue going straight to switch
					myRobot.driveCartesian(0, defaultSpeed, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRF > distSwitch) {
					stepNumber = 3;
					leftFront.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 3) {
				if (straight_encRevsLF < 45) { // turn towards switch
					myRobot.driveCartesian(0, 0, .4);
				} else if (straight_encRevsLF >= 45) {
					stepNumber = 4;
				}
			} else if (stepNumber == 4) {
				myRobot.driveCartesian(0, 0, 0);
				if (encRevsElevator > 27000) {
					intakeMotor1.set(ControlMode.PercentOutput, -releaseCube);
					intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
			/*
			 * if (elevSwitchHi.get()) { elevatorExtension1.set(0); } else {
			 * elevatorExtension1.set(ControlMode.MotionMagic, 28000); }
			 */
		}

		else if (gameData.charAt(0) != 'L' && gameData.charAt(1) == 'L') { // going for scale
			if (stepNumber == 1) { // jerk motion
				if (straight_encRevsRF < 10) { // go straight to scale
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(timerDelay);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= distScale) { // continue going straight to scale
					myRobot.driveCartesian(0, defaultSpeed, 0);
				} else if (straight_encRevsRF > distScale) {
					stepNumber = 3;
					leftFront.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 3) {
				if (straight_encRevsLF < 45) { // turn towards scale
					myRobot.driveCartesian(0, 0, .4);
				} else if (straight_encRevsLF >= 45) {
					rightBack.setSelectedSensorPosition(0, 0, 10);
					stepNumber = 4;
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 4) { // back away from scale
				/*
				 * if (elevSwitchHi.get()) { elevatorExtension1.set(0); } else {
				 * elevatorExtension1.set(ControlMode.MotionMagic, 60000); }
				 */
				if (straight_encRevsRB > -15) {
					rightBack.getSelectedSensorPosition(0);
					myRobot.driveCartesian(0, -defaultSpeed, 0);
				} else if (Math.abs(encRevsElevator) > 59000) {
					stepNumber = 5;
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
				/*
				 * } else if (stepNumber == 5) { intakeMotor1.set(ControlMode.PercentOutput,
				 * -releaseCube); intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				 */
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
		}

		else if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') { // going for switch
			if (stepNumber == 1) { // jerk motion
				if (straight_encRevsRF < 10) { // go straight to switch
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(timerDelay);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= distSwitch) { // continue going straight to switch
					myRobot.driveCartesian(0, defaultSpeed, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRF > distSwitch) {
					stepNumber = 3;
					leftFront.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 3) {
				if (straight_encRevsLF < 45) { // turn towards switch
					myRobot.driveCartesian(0, 0, .4);
				} else if (straight_encRevsLF >= 45) {
					stepNumber = 4;
				}
			} else if (stepNumber == 4) {
				myRobot.driveCartesian(0, 0, 0);
				if (encRevsElevator > 27000) {
					intakeMotor1.set(ControlMode.PercentOutput, -releaseCube);
					intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
			/*
			 * if (elevSwitchHi.get()) { elevatorExtension1.set(0); } else {
			 * elevatorExtension1.set(ControlMode.MotionMagic, 28000); }
			 */
		}

		else if (gameData.charAt(0) != 'L' && gameData.charAt(1) != 'L') { // crossing autoline
			if (stepNumber == 1) { // jerk code
				if (straight_encRevsRB < 10) { // go straight toward autoline
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRB >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(.3);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsLF <= 140) { // continue going straight toward autoline
					myRobot.driveCartesian(0, defaultSpeed, 0);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
		} else {
			myRobot.driveCartesian(0, 0, 0);
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
	}

	public void CrossLine() {
		if (straight_encRevsLF <= 140) {
			myRobot.driveCartesian(0, defaultSpeed, 0);
		} else {
			myRobot.driveCartesian(0, 0, 0);
		}
	}

	public void RightGen() { // starting configuration is back bumper on wall
		SmartDashboard.putNumber("Right Front Encoder", straight_encRevsRF);
		SmartDashboard.putNumber("Right Back Encoder", straight_encRevsRB);
		SmartDashboard.putNumber("Left Front Encoder", straight_encRevsLF);
		SmartDashboard.putNumber("Step Number", stepNumber);
		SmartDashboard.putNumber("Program", switBinFin);

		if (gameData.charAt(0) != 'R' && gameData.charAt(1) == 'R') { // going for scale
			if (stepNumber == 1) { // jerk code
				if (straight_encRevsRF < 10) { // go straight toward scale
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(timerDelay);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF < 285) { // continue going straight toward scale
					myRobot.driveCartesian(0, defaultSpeed, -.01);
				} else if (straight_encRevsRF >= 285) {
					stepNumber = 3;
					rightBack.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 3) { // turn toward scale
				SmartDashboard.putNumber("Right Rear Encoder", rightBack.getSelectedSensorPosition(0));
				SmartDashboard.putNumber("Step Number", stepNumber);
				if (Math.abs(straight_encRevsRB) < 45) {
					myRobot.driveCartesian(0, 0, -defaultSpeed);
				} else if (Math.abs(straight_encRevsRB) >= 45) {
					myRobot.driveCartesian(0, 0, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
					stepNumber = 4;
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 4) { // back away from scale
				/*
				 * if (elevSwitchHi.get()) { elevatorExtension1.set(0); } else {
				 * elevatorExtension1.set(ControlMode.MotionMagic, 60000);
				 */
				if (straight_encRevsLF > -15) {
					leftFront.getSelectedSensorPosition(0);
					myRobot.driveCartesian(0, -defaultSpeed, 0);
				} else if (Math.abs(encRevsElevator) > 59000) {
					stepNumber = 5;
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 5) {
				// intakeMotor1.set(ControlMode.PercentOutput, -releaseCube);
				// intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
		}

		else if (gameData.charAt(0) == 'R' && gameData.charAt(1) != 'R') { // going for switch
			if (stepNumber == 1) { // jerk motion
				if (straight_encRevsRF < 10) { // go straight to switch
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(.1);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= distSwitch) { // continue going straight to switch
					myRobot.driveCartesian(0, defaultSpeed, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRF > distSwitch) {
					stepNumber = 3;
					leftFront.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 3) {
				if (straight_encRevsLF > -20) { // turn towards switch
					myRobot.driveCartesian(0, 0, -.4);
				} else if (straight_encRevsLF <= -20) {
					stepNumber = 4;
				}
			} else if (stepNumber == 4) {
				myRobot.driveCartesian(0, 0, 0);
				if (encRevsElevator > 27000) {
					// intakeMotor1.set(ControlMode.PercentOutput, -releaseCube);
					// intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
			/*
			 * if (elevSwitchHi.get()) { elevatorExtension1.set(0); } else {
			 * elevatorExtension1.set(ControlMode.MotionMagic, 28000); }
			 */
		}

		else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') { // going for switch
			if (stepNumber == 1) { // jerk motion
				if (straight_encRevsRF < 10) { // go straight to switch
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(.1);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= distSwitch) { // continue going straight to switch
					myRobot.driveCartesian(0, defaultSpeed, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRF > distSwitch) {
					stepNumber = 3;
					leftFront.setSelectedSensorPosition(0, 0, 10);
				}
			} else if (stepNumber == 3) {
				if (straight_encRevsLF > -20) { // turn towards switch
					myRobot.driveCartesian(0, 0, -.4);
				} else if (straight_encRevsLF <= -20) {
					stepNumber = 4;
				}
			} else if (stepNumber == 4) {
				myRobot.driveCartesian(0, 0, 0);
				if (encRevsElevator > 27000) {
					// intakeMotor1.set(ControlMode.PercentOutput, -releaseCube);
					// intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
			/*
			 * if (elevSwitchHi.get()) { elevatorExtension1.set(0); } else {
			 * elevatorExtension1.set(ControlMode.MotionMagic, 28000); }
			 */
		}

		else if (gameData.charAt(0) != 'R' && gameData.charAt(1) != 'R') { // cross autoline
			if (stepNumber == 1) { // jerk code
				if (straight_encRevsRF < 10) { // go straight toward switch
					myRobot.driveCartesian(0, .6, 0);
				} else if (straight_encRevsRF >= 10) {
					myRobot.driveCartesian(0, 0, 0);
					Timer.delay(timerDelay);
					stepNumber = 2;
				} else {
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= 140) {
					myRobot.driveCartesian(0, defaultSpeed, 0);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
		} else { // do nothing
			myRobot.driveCartesian(0, 0, 0);
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
	}

	@Override
	public void teleopInit() {
		releaseCube = -.5;
		defaultSpeed = .5;

		CameraServer.getInstance().startAutomaticCapture(0); // number is USB port on RIO, we tink
	}

	@Override
	public void teleopPeriodic() {

		SmartDashboard.putBoolean("Elevator Switch High", elevSwitchHi.get());

		myRobot.driveCartesian(joystick.getRawAxis(0) / 1.1, -joystick.getRawAxis(1) / 1.1,
				joystick.getRawAxis(2) / 1.1, 0);

		if (joystick.getRawButton(5)) {
			leftFront.setSelectedSensorPosition(0, 0, 10);
			leftBack.setSelectedSensorPosition(0, 0, 10);
			rightFront.setSelectedSensorPosition(0, 0, 10);
			rightBack.setSelectedSensorPosition(0, 0, 10);
			elevatorExtension1.setSelectedSensorPosition(0, 0, 10);
			navx.zeroYaw();
		} else {
			strafe_encRevsLF = leftFront.getSelectedSensorPosition(0);
			strafe_encRevsRF = rightFront.getSelectedSensorPosition(0);
			strafe_encRevsRB = rightBack.getSelectedSensorPosition(0);
			encRevsElevator = elevatorExtension1.getSelectedSensorPosition(0);
		}

		// cube mechanism
		if (xbox.getRawAxis(3) > 0 && xbox.getRawButton(6)) {
			intakeMotor1.configPeakOutputForward(100, 10);
			intakeMotor1.configPeakOutputReverse(-100, 10);
			intakeMotor2.configPeakOutputForward(100, 10);
			intakeMotor2.configPeakOutputReverse(-100, 10);
			intakeMotor1.set(ControlMode.PercentOutput, joystick.getRawAxis(1) * 2);
			intakeMotor2.set(ControlMode.PercentOutput, -joystick.getRawAxis(1) * 2);
		} else if (xbox.getRawAxis(3) > 0 && !xbox.getRawButton(6)) {
			intakeMotor1.set(ControlMode.PercentOutput, -0.6);
			intakeMotor2.set(ControlMode.PercentOutput, 0.6);
		} else if (xbox.getRawAxis(2) > 0) {
			intakeMotor1.set(ControlMode.PercentOutput, 0.6);
			intakeMotor2.set(ControlMode.PercentOutput, -0.6);
		} else {
			intakeMotor1.set(ControlMode.PercentOutput, 0);
			intakeMotor2.set(ControlMode.PercentOutput, 0);
		}

		SmartDashboard.putNumber("Intake Motor 1 Output", intakeMotor1.getMotorOutputPercent());
		SmartDashboard.putNumber("Intake Motor 2 Output", intakeMotor2.getMotorOutputPercent());

		// Climbing
		if (xbox.getRawButton(4)) { // button 4 is the Y button -- hang
			hangMotor1.set(ControlMode.PercentOutput, .5);
			hangMotor2.set(ControlMode.PercentOutput, .5);
		} else if (xbox.getRawButton(3)) { // button 3 is the X button -- come down from hang
			hangMotor1.set(ControlMode.PercentOutput, -.5);
			hangMotor2.set(ControlMode.PercentOutput, -.5);
		} else { // do nothing
			hangMotor1.set(ControlMode.PercentOutput, 0);
			hangMotor2.set(ControlMode.PercentOutput, 0);
		}

		// Elevator
		double switchHeight = 30; // heights for presets in inches
		double scaleMid = 70; // scale at level position

		if (elevSwitchHi.get()) {
			if (xbox.getRawAxis(5) > .1) {
				elevatorExtension1.set(ControlMode.PercentOutput, -xbox.getRawAxis(5) / 2);
				elevatorExtension2.set(ControlMode.PercentOutput, -xbox.getRawAxis(5) / 2);
			} else if (xbox.getRawAxis(5) <= 0) {
				elevatorExtension1.set(ControlMode.PercentOutput, 0);
				elevatorExtension2.set(ControlMode.PercentOutput, 0); 
			}
		} /*else if (elevSwitchLo.get()) {
			if (xbox.getRawAxis(5) < -.1) {
				elevatorExtension1.set(ControlMode.PercentOutput, -xbox.getRawAxis(5) / 2);
				elevatorExtension2.set(ControlMode.PercentOutput, -xbox.getRawAxis(5) / 2);
			} else if (xbox.getRawAxis(5) >= 0) {
				elevatorExtension1.set(ControlMode.PercentOutput, 0);
				elevatorExtension2.set(ControlMode.PercentOutput, 0);
			}
		} */ else {  
			if (xbox.getRawButton(5)) {
				elevatorExtension1.set(ControlMode.MotionMagic, 23000);
			} else {
				if (xbox.getRawAxis(5) > .1 || xbox.getRawAxis(5) < -.1) {
					elevatorExtension1.set(ControlMode.PercentOutput, -xbox.getRawAxis(5) / 2);
					elevatorExtension2.set(ControlMode.PercentOutput, -xbox.getRawAxis(5) / 2);
				} else {
					elevatorExtension1.set(ControlMode.PercentOutput, 0);
					elevatorExtension2.set(ControlMode.PercentOutput, 0);
				}
			}
		}

		// SmartDashboard values

		// encoder values
		SmartDashboard.putNumber("Left Front Encoder", strafe_encRevsLF);
		SmartDashboard.putNumber("Left Rear Encoder", leftBack.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Right Front Encoder", strafe_encRevsRF);
		SmartDashboard.putNumber("Right Rear Encoder", rightBack.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Elevator Encoder", encRevsElevator);

		// gyro values
		SmartDashboard.putNumber("Gyro Yaw", navx.getYaw());
		SmartDashboard.putNumber("Gyro Angle", navx.getAngle());

		// drivetrain voltage
		double leftFrontVoltage = leftFront.getMotorOutputVoltage();
		double leftBackVoltage = leftBack.getMotorOutputVoltage();
		double rightFrontVoltage = rightFront.getMotorOutputVoltage();
		double rightBackVoltage = rightBack.getMotorOutputVoltage();
		SmartDashboard.putNumber("Left Front Voltage", leftFrontVoltage);
		SmartDashboard.putNumber("Left Back Voltage", leftBackVoltage);
		SmartDashboard.putNumber("Right Front Voltage", rightFrontVoltage);
		SmartDashboard.putNumber("Right Back Voltage", rightBackVoltage);

		// elevator voltage outputs
		double elevatorExt1Voltage = elevatorExtension1.getMotorOutputVoltage();
		double elevatorExt2Voltage = elevatorExtension2.getMotorOutputVoltage();

		SmartDashboard.putNumber("Elevator Velocity", elevatorExtension2.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Elevator 1 Voltage", elevatorExt1Voltage);
		SmartDashboard.putNumber("Elevator 2 Voltage", elevatorExt2Voltage);
	}

	@Override
	public void disabledPeriodic() {
		strafe_encRevsLF = leftFront.getSelectedSensorPosition(0);
		strafe_encRevsRF = rightFront.getSelectedSensorPosition(0);
		// strafe_encRevsLB = leftBack.getSelectedSensorPosition(0);
		strafe_encRevsRB = rightBack.getSelectedSensorPosition(0);

		SmartDashboard.putNumber("Left Front Encoder", strafe_encRevsLF);
		// SmartDashboard.putNumber("Left Rear Encoder", strafe_encRevsLB);
		SmartDashboard.putNumber("Right Front Encoder", strafe_encRevsRF);
		SmartDashboard.putNumber("Right Rear Encoder", strafe_encRevsRB);

	}

	@Override
	public void testPeriodic() {
		LiveWindow.addActuator("Drivetrain", "Right Front", rightFront);
		LiveWindow.addActuator("Drivetrain", "Right Back", rightBack);
		LiveWindow.addActuator("Drivetrain", "Left Front", leftFront);
		LiveWindow.addActuator("Drivetrain", "Left Back", leftBack);

		LiveWindow.addActuator("Hanger", "Hanging Motor 1", hangMotor1);
		LiveWindow.addActuator("Hanger", "Hanging Motor 2", hangMotor2);

		LiveWindow.addActuator("Elevator", "Elevator Motor 1", elevatorExtension1);
		LiveWindow.addActuator("Elevator", "Elevator Motor 2", elevatorExtension2);
	}
}