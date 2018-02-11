/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team433.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	// Drive train
	MecanumDrive myRobot;
	WPI_TalonSRX leftFront = new WPI_TalonSRX(2);
	WPI_TalonSRX leftBack = new WPI_TalonSRX(3);
	WPI_TalonSRX rightFront = new WPI_TalonSRX(4);
	WPI_TalonSRX rightBack = new WPI_TalonSRX(5);

	double straight_encRevsLF;
	double straight_encRevsLB;
	double straight_encRevsRF;
	double straight_encRevsRB;

	double strafe_encRevsLF;
	double strafe_encRevsLB;
	double strafe_encRevsRF;
	double strafe_encRevsRB;

	Joystick joystick = new Joystick(0);
	Joystick xbox = new Joystick(1);

	// Elevator
	WPI_TalonSRX elevatorExtension = new WPI_TalonSRX(6);
	VictorSPX intakeMotor1 = new VictorSPX(0);
	VictorSPX intakeMotor2 = new VictorSPX(1);
	double encRevsElevator; // encoder for elevator

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

	// Extra Talon 7
	WPI_TalonSRX extraTalon = new WPI_TalonSRX(7);

	@Override
	public void robotInit() {
		myRobot = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
		navx = new AHRS(SerialPort.Port.kUSB);

		leftFront.setSensorPhase(true);
		leftBack.setSensorPhase(true);
	}

	@Override
	public void autonomousInit() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		defaultSpeed = 0.6;
		releaseCube = -0.6;
		distScale = 324; // From drive station wall to scale's null territory
		distPlatZone = 227; // From drive station wall to in between switch and
							// platforms
		distSwitch = 168; // From drive station wall to horizontal side of scale
							// (side perpendicular to
							// drive station)
		distCross = 200; // From one side of switch to other
		distPlatToScale = 97; // To get from distPlatZone to scale's null
								// territory
		stepNumber = 1;
		navx.zeroYaw();
	}

	@Override
	public void autonomousPeriodic() {

		// divided by 259 to convert encoder loops to inches
		straight_encRevsLF = leftFront.getSelectedSensorPosition(0) / 259;
		straight_encRevsLB = leftBack.getSelectedSensorPosition(0) / 259;
		straight_encRevsRF = rightFront.getSelectedSensorPosition(0) / 259;
		straight_encRevsRB = rightBack.getSelectedSensorPosition(0) / 259;

		strafe_encRevsLF = leftFront.getSelectedSensorPosition(0);
		strafe_encRevsLB = leftBack.getSelectedSensorPosition(0);
		strafe_encRevsRF = rightFront.getSelectedSensorPosition(0);
		strafe_encRevsRB = rightBack.getSelectedSensorPosition(0);

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
			RightGen(); // yellow on
			break;
		}
	}

	public void DoNothing() { // do nothing
		myRobot.driveCartesian(0, 0, 0);
	}

	public void LeftScale() { // starting configuration left side touching
								// driver wall
		if (gameData.charAt(1) == 'R') { // going for right side of scale
			if (stepNumber == 1) {
				if (strafe_encRevsRF > -distPlatZone) { // strafe right to
														// platform zone
														// (between switch and
														// scale)
					myRobot.driveCartesian(defaultSpeed, 0, 0);
				} else if (strafe_encRevsRF <= -distPlatZone) {
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
					leftBack.setSelectedSensorPosition(0, 0, 10);
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 3) {
				if (strafe_encRevsLB > -distPlatToScale) { // drive sideways to
															// scale
					leftBack.getSelectedSensorPosition(0);
					myRobot.driveCartesian(defaultSpeed, 0, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsLB <= -distPlatToScale) {
					leftFront.getSelectedSensorPosition(0);
					myRobot.driveCartesian(0, 0, 0);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
		} else if (gameData.charAt(1) == 'L') {
			if (strafe_encRevsLF < distScale) { // strafe right until at scale
				myRobot.driveCartesian(defaultSpeed, 0, 0);
				navx.zeroYaw();
			} else if (straight_encRevsLF >= distScale && Math.abs(navx.getAngle()) <= 130) { // turn
																								// 180
																								// to
																								// face
																								// scale
				rightFront.set(0.4);
				rightBack.set(0.4);
				leftFront.set(0.4);
				leftBack.set(0.4);
			} else if (Math.abs(navx.getAngle()) >= 130) { // place cube
				myRobot.driveCartesian(0, 0, 0);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
		}
	}

	public void CenterSwitch() { // starting configuration back bumper touching
									// wall and left bumper aligned with
									// exchange zone
		if (gameData.charAt(0) == 'L') {
			if (stepNumber == 1) {
				if (straight_encRevsLB < 24) { // move forward past exchange
												// zone ramp
					myRobot.driveCartesian(0, defaultSpeed, 0);
					leftFront.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsLB >= 24 && strafe_encRevsLF < 61) { // move
																				// sideways
																				// to
																				// switch
																				// plate
					leftFront.getSelectedSensorPosition(0);
					myRobot.driveCartesian(-defaultSpeed, 0, 0);
					rightFront.setSelectedSensorPosition(0, 0, 10);
				} else if (strafe_encRevsLF >= 61 && straight_encRevsRF < 116) {
					stepNumber = 2;
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF < 116) { // move forward to switch
					rightFront.getSelectedSensorPosition(0);
					myRobot.driveCartesian(0, defaultSpeed, 0);
				} else if (straight_encRevsRF >= 116) { // place cube
					intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
					intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
		} else if (gameData.charAt(0) == 'R') {
			if (strafe_encRevsLF < 48) { // strafing right until past pyramid
				myRobot.driveCartesian(defaultSpeed, 0, 0);
				rightFront.setSelectedSensorPosition(0, 0, 10);
			} else if (strafe_encRevsLF >= 48 && straight_encRevsRF < 140) { // forward
																				// to
																				// switch
				rightFront.getSelectedSensorPosition(0);
				myRobot.driveCartesian(0, defaultSpeed, 0);
			} else if (straight_encRevsRF >= 140) { // place cube
				intakeMotor1.set(ControlMode.PercentOutput, defaultSpeed);
				intakeMotor2.set(ControlMode.PercentOutput, defaultSpeed);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}

		} else {
			myRobot.driveCartesian(0, 0, 0);
		}
	}

	public void LeftSwitch() { // starting configuration is backwards
		if (gameData.charAt(0) == 'R') {
			if (straight_encRevsLF >= -distPlatZone) { // backwards past switch
				myRobot.driveCartesian(0, -defaultSpeed, 0);
				rightFront.setSelectedSensorPosition(0, 0, 10);
			} else if (strafe_encRevsRF < distCross && straight_encRevsLF < -distPlatZone) { // across
																								// to
																								// scale
																								// platform
				rightFront.getSelectedSensorPosition(0);
				myRobot.driveCartesian(-defaultSpeed, 0, 0);
			} else if (strafe_encRevsRF >= distCross) { // place cube
				myRobot.driveCartesian(0, 0, 0);
				stepNumber = 2;
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
		} else if (gameData.charAt(0) == 'L') {
			if (straight_encRevsLF >= -distSwitch) { // back up to switch
				myRobot.driveCartesian(0, -defaultSpeed, 0);
				navx.zeroYaw();
			} else if (navx.getAngle() > -62 && straight_encRevsLF < -distSwitch) { // turn
																					// 90
																					// degrees
																					// left
				leftFront.set(-0.4);
				rightFront.set(-0.4);
				leftBack.set(-0.4);
				rightBack.set(-0.4);
			} else if (navx.getAngle() <= -62) { // place cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}

		} else {
			myRobot.driveCartesian(0, 0, 0);
		}
	}

	public void RightScale() { // starting configuration is right side on wall
		if (gameData.charAt(1) == 'R') {

			if (strafe_encRevsLB < distScale) { // strafe left to scale
				myRobot.driveCartesian(-defaultSpeed, 0, 0);
			} else if (strafe_encRevsLB >= distScale && Math.abs(navx.getAngle()) <= 162) { // 180
																							// degree
																							// turn
				rightFront.set(0.4);
				rightBack.set(0.4);
				leftFront.set(0.4);
				leftBack.set(0.4);
			} else if (Math.abs(navx.getAngle()) > 162) { // place cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
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
					leftBack.setSelectedSensorPosition(0, 0, 10);
				} else if (straight_encRevsRF <= -distCross && strafe_encRevsLB < distPlatToScale) {
					stepNumber = 2;
				}
			} else if (stepNumber == 2) {
				if (straight_encRevsRF <= -distCross && strafe_encRevsLB < distPlatToScale) { // strafe
																								// left
																								// to
																								// line
																								// up
																								// with
																								// scale
					leftBack.getSelectedSensorPosition(0);
					myRobot.driveCartesian(-defaultSpeed, 0, 0);
				} else if (strafe_encRevsLB >= distPlatToScale) { // place cube
					intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
					intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
				} else {
					myRobot.driveCartesian(0, 0, 0);
				}
			}
		}
	}

	public void RightSwitch() { // starting configuration is backwards
		if (gameData.charAt(0) == 'L') {
			if (straight_encRevsLF >= -distPlatZone) {
				myRobot.driveCartesian(0, -defaultSpeed, 0);
				rightFront.setSelectedSensorPosition(0, 0, 10);
			} else if (strafe_encRevsRF <= 156 && straight_encRevsLF < -distPlatZone) {
				rightFront.getSelectedSensorPosition(0);
				myRobot.driveCartesian(defaultSpeed, 0, 0);
			} else if (strafe_encRevsRF > 156 && straight_encRevsLF < -distPlatZone) { // place
																						// cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
		} else if (gameData.charAt(0) == 'R') {
			if (straight_encRevsLF >= -distSwitch) {
				myRobot.driveCartesian(0, -defaultSpeed, 0);
				navx.zeroYaw();
			} else if (straight_encRevsLF < -distSwitch && navx.getYaw() <= 90) {
				myRobot.driveCartesian(0, 0, defaultSpeed);
			} else if (navx.getYaw() > 90) { // place cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}

		}
	}

	public void LeftGen() { // starting configuration is right side on wall...?
		if (gameData.charAt(0) == 'L' && gameData.charAt(1) != 'L') {
			if (strafe_encRevsRF <= distSwitch) {
				myRobot.driveCartesian(-defaultSpeed, 0, 0);
				leftFront.setSelectedSensorPosition(0, 0, 10);
			} else if (straight_encRevsLF <= 12 && strafe_encRevsRF > distSwitch) {
				leftFront.getSelectedSensorPosition(0);
				myRobot.driveCartesian(0, defaultSpeed, 0);
			} else if (straight_encRevsLF > 12 && strafe_encRevsRF > distSwitch) { // place
																					// cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
		} else if (gameData.charAt(0) != 'L' && gameData.charAt(1) == 'L') { // if
																				// going
																				// to
																				// left
																				// side
																				// of
																				// scale
																				// (from
																				// left
																				// position)
																				// starting
																				// with
																				// right
																				// side
																				// of
																				// robot
																				// against
																				// DS
			if (strafe_encRevsRF <= distScale) {
				myRobot.driveCartesian(-defaultSpeed, 0, 0);
			} else if (strafe_encRevsRF > distScale) { // place cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
		} else if (gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') { // if
																				// going
																				// to
																				// left
																				// switch
																				// starting
																				// with
																				// right
																				// side
																				// on
																				// DS
			if (strafe_encRevsRF <= distSwitch) {
				myRobot.driveCartesian(-defaultSpeed, 0, 0);
				straight_encRevsLF = 0;
			} else if (straight_encRevsLF <= 12) {
				leftFront.getSelectedSensorPosition(0);
				myRobot.driveCartesian(0, defaultSpeed, 0);
			} else if (straight_encRevsLF > 12) { // place cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else {
				myRobot.driveCartesian(0, 0, 0);

			}
		} else if (gameData.charAt(0) != 'L' && gameData.charAt(1) != 'L') {
			if (strafe_encRevsLF <= 140) {
				myRobot.driveCartesian(0, defaultSpeed, 0);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
		}
	}

	public void CrossLine() {
		if (straight_encRevsLF <= 140) {
			myRobot.driveCartesian(0, defaultSpeed, 0);
		} else {
			myRobot.driveCartesian(0, 0, 0);
		}
	}

	public void RightGen() { // starting configuration is left bumper on wall
		if (gameData.charAt(0) != 'R' && gameData.charAt(1) == 'R') { // if
																		// scale
																		// is
																		// right
			if (strafe_encRevsRF > distScale) { // strafe to scale
				myRobot.driveCartesian(defaultSpeed, 0, 0);
			} else if (strafe_encRevsRF >= distScale) { // place cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else { // do nothing
				myRobot.driveCartesian(0, 0, 0);
			}
		} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) != 'R') {// if
																			// switch
																			// is
																			// right
			if (strafe_encRevsRF < distSwitch) { // strafe until at switch
				myRobot.driveCartesian(defaultSpeed, 0, 0);
				leftFront.setSelectedSensorPosition(0, 0, 10);
			} else if (strafe_encRevsRF >= distSwitch && straight_encRevsLF < 12) { // move
																					// forward
																					// to
																					// switch
				leftFront.getSelectedSensorPosition(0);
				myRobot.driveCartesian(0, defaultSpeed, 0);
			} else if (strafe_encRevsRF >= distSwitch && straight_encRevsLF >= 12) { // place
																						// cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else { // do nothing
				myRobot.driveCartesian(0, 0, 0);
			}
		} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') { // if
																				// both
																				// are
																				// right
			if (strafe_encRevsRF < distSwitch) { // strafe until at switch
				myRobot.driveCartesian(defaultSpeed, 0, 0);
				leftFront.setSelectedSensorPosition(0, 0, 10);
			} else if (strafe_encRevsRF >= distSwitch && straight_encRevsLF < 12) { // move
																					// forward
																					// to
																					// switch
				leftFront.getSelectedSensorPosition(0);
				myRobot.driveCartesian(0, defaultSpeed, 0);
			} else if (strafe_encRevsRF >= distSwitch && straight_encRevsLF >= 12) { // place
																						// cube
				intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
				intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
			} else { // do nothing
				myRobot.driveCartesian(0, 0, 0);
			}
		} else if (gameData.charAt(0) != 'R' && gameData.charAt(1) != 'R') { // if
																				// neither
																				// are
																				// right
			if (strafe_encRevsRF <= 140) {
				myRobot.driveCartesian(defaultSpeed, 0, 0);
			} else {
				myRobot.driveCartesian(0, 0, 0);
			}
		} else { // do nothing
			myRobot.driveCartesian(0, 0, 0);
		}
	}

	// TeleOp
	@Override
	public void teleopInit() {

	}

	@Override
	public void teleopPeriodic() {

		if (joystick.getRawAxis(0) < -.1 || joystick.getRawAxis(0) > .1) {
			myRobot.driveCartesian(joystick.getRawAxis(0), joystick.getRawAxis(1), 0, 0);
		} else if (joystick.getRawAxis(1) < -.1 || joystick.getRawAxis(1) > .1) {
			myRobot.driveCartesian(joystick.getRawAxis(0), joystick.getRawAxis(1), 0, 0);
		} else if (joystick.getRawAxis(2) < -.1 || joystick.getRawAxis(2) > .1) {
			myRobot.driveCartesian(joystick.getRawAxis(0), joystick.getRawAxis(1), 0, 0);
		} else {
			myRobot.driveCartesian(0, 0, 0, 0);
		}

		// encoder reset
		if (joystick.getRawButton(5)) {
			leftFront.setSelectedSensorPosition(0, 0, 10);
			leftBack.setSelectedSensorPosition(0, 0, 10);
			rightFront.setSelectedSensorPosition(0, 0, 10);
			rightBack.setSelectedSensorPosition(0, 0, 10);
			elevatorExtension.setSelectedSensorPosition(0, 0, 10);
		} else {
			strafe_encRevsLF = leftFront.getSelectedSensorPosition(0);
			strafe_encRevsRF = rightFront.getSelectedSensorPosition(0);
			strafe_encRevsLB = leftBack.getSelectedSensorPosition(0);
			strafe_encRevsRB = rightBack.getSelectedSensorPosition(0);
			encRevsElevator = elevatorExtension.getSelectedSensorPosition(0);
		}

		// cube intake
		if (xbox.getRawAxis(3) > 0) {
			intakeMotor1.set(ControlMode.PercentOutput, defaultSpeed);
			intakeMotor2.set(ControlMode.PercentOutput, defaultSpeed);
		}
		// cube output
		else if (xbox.getRawAxis(2) > 0) {
			intakeMotor1.set(ControlMode.PercentOutput, releaseCube);
			intakeMotor2.set(ControlMode.PercentOutput, releaseCube);
		}

		// Climbing
		if (xbox.getRawAxis(1) < 0) {
			// extend climb mechanism
		}

		if (xbox.getRawButton(4)) { // button 4 is the Y button
			// clamp on climb bar
		}

		// Elevator
		double switchHeight = 30; // heights for presets in inches
		double scaleMid = 70;

		// elevator extension
		if (xbox.getRawAxis(5) != 0) {
			elevatorExtension.set(xbox.getRawAxis(5));
		} else {
			elevatorExtension.set(0);
		}

		/*
		 * // switch preset else if (xbox.getRawButton(2)) { // button 2 is the
		 * B button if (encRevsElevator > switchHeight) {
		 * elevatorExtension.set(-.3); } else if (encRevsElevator <=
		 * switchHeight) { elevatorExtension.set(.3); } // mid-scale preset }
		 * else if (xbox.getRawButton(3)) { // button 3 is X button if
		 * (encRevsElevator > scaleMid) { elevatorExtension.set(-.3); } else if
		 * (encRevsElevator <= scaleMid) { elevatorExtension.set(.3); }
		 */

		// SmartDashboard values
		SmartDashboard.putNumber("Left Front Encoder", strafe_encRevsLF);
		SmartDashboard.putNumber("Left Rear Encoder", strafe_encRevsLB);
		SmartDashboard.putNumber("Right Front Encoder", strafe_encRevsRF);
		SmartDashboard.putNumber("Right Rear Encoder", strafe_encRevsRB);
		SmartDashboard.putNumber("Elevator Encoder", encRevsElevator);

		SmartDashboard.putNumber("Gyro Yaw", navx.getYaw());
		SmartDashboard.putNumber("Gyro Angle", navx.getAngle());
	}

	@Override
	public void testPeriodic() {
		LiveWindow.addActuator("Drivetrain", "Right Front", rightFront);
		LiveWindow.addActuator("Drivetrain", "Right Back", rightBack);
		LiveWindow.addActuator("Drivetrain", "Left Front", leftFront);
		LiveWindow.addActuator("Drivetrain", "Left Back", leftBack);

		LiveWindow.addActuator("Hanger", "Hanging Motor 1", hangMotor1);
		LiveWindow.addActuator("Hanger", "Hanging Motor 2", hangMotor2);

		LiveWindow.addActuator("Elevator", "Elevator", elevatorExtension);
	}
}
