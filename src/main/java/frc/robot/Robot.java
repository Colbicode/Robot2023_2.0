// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Team 3288 robot
 * Programed by Brandon, Colby, and Mr. N
 */

package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * Base code from WPILib don't change
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //The code above is WPIlib


  //The controllers. Blue controller is driver. Red is operator.
  private final GenericHID redController = new GenericHID(1);
  private final GenericHID blueController = new GenericHID(0);

  
  //Drive motors
  private final CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private final MotorControllerGroup leftDriveMotors = new MotorControllerGroup(leftMotor1, leftMotor2);

  private final CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final MotorControllerGroup rightDriveMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
    //Encoders
  private final RelativeEncoder leftMotor1Encoder = leftMotor1.getEncoder();
  private final RelativeEncoder leftMotor2Encoder = leftMotor2.getEncoder();
  private final RelativeEncoder rightMotor1Encoder = rightMotor1.getEncoder();
  private final RelativeEncoder rightMotor2Encoder = rightMotor2.getEncoder();
  

  //Drive Train
  private final DifferentialDrive driveTrain = new DifferentialDrive(leftDriveMotors, rightDriveMotors);

  //DoubleSolenoids on the big arm. "firstStage" is the big one and "secondStage" is the small one. 
  //"clamp" will control the clamping motion of the intake
  private final DoubleSolenoid firstStage = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
  private final DoubleSolenoid secondStage = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 4);
  private final DoubleSolenoid clamp = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 6);

  //Motor for exstention of the arm.
  private final CANSparkMax armExtensionMotor = new CANSparkMax(5, MotorType.kBrushed);
    //Encoder
  private final RelativeEncoder armExtensionEncoder = armExtensionMotor.getEncoder();

  //Intake motors
  private final CANSparkMax tooth1 = new CANSparkMax(6, MotorType.kBrushed);
  private final CANSparkMax tooth2 = new CANSparkMax(7, MotorType.kBrushed);
  private final MotorControllerGroup teeth = new MotorControllerGroup(tooth1, tooth2);

  //provides the status of the intake. False is open, and true is closed. Starts in closed position to hold game piece
  public boolean intakeStatus = true;

  //Distance of middle scoring level
  public final double middleNodeDistance = 0;

  //Distance of the high node
  public final double highNodeDistance = 1;

  //sets the distance to travel to get out of community in autonomous
  public final double distanceOutOfCommunity = 30; //subject to change.

  //set the distance to travel from out of community to balance
  public final double distanceToBalance = 15; //subject to change.

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //This is where we change the setInverted properties on the motors.
    leftMotor1.setInverted(true); //True or false depending on when we test the motors.
    leftMotor2.setInverted(true);
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);

    armExtensionMotor.setInverted(true);
    tooth1.setInverted(false);
    tooth2.setInverted(true);

    //Makes the intake closed at the start of the match
    intakeStatus = true;

    //Make sure that it is in starting configuration.
    armLevel(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  public void scoreDriveBack() {
    armLevel(2); //arm to scoring level
    
    while((armExtensionEncoder.getPosition() < highNodeDistance) && (Timer.getFPGATimestamp() < 3)) { //While the arm is shorter than the high node distance and time is more than 3 seconds.
      armExtensionMotor.set(0.75); //Extends arm at speed of 0.75.
    }

    bite(false);
    
    while((armExtensionEncoder.getPosition() < 1) && (Timer.getFPGATimestamp() < 6 )) { //retracts the arm until encoder postion is less than or time is 6sec.
      armExtensionMotor.set(-0.75);
    }

    while((leftMotor1Encoder.getPosition() < distanceOutOfCommunity) && (Timer.getFPGATimestamp() < 12)) { //drives backward out of the community.
      driveTrain.arcadeDrive(-0.75, 0);
    }
  }

  public void scoreDriveBackBalance() {
    scoreDriveBack();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //Sets motors IdleMode to coast. Need to add 3 second wait. Figure it out, take your time. Maybe use boolean for brakes.
    leftMotor1.setIdleMode(IdleMode.kCoast);
    leftMotor2.setIdleMode(IdleMode.kCoast);
    rightMotor1.setIdleMode(IdleMode.kCoast);
    rightMotor2.setIdleMode(IdleMode.kCoast);
  }

  //Up and down motion of the arm. Need to figure out how to start with this.
  public void armLevel(int level) {
    // 0 starting configeration, 1 ground level, 2 scoring level.
    switch(level){
      case 0: //Starting configuration
        firstStage.set(Value.kForward);
        secondStage.set(Value.kForward);
        break;
      case 1: //Ground Level Configuration
        firstStage.set(Value.kReverse);
        secondStage.set(Value.kReverse);
      case 2: //Scoring Level Configuration
        firstStage.set(Value.kForward);
        firstStage.set(Value.kReverse);
        break;
      default:
        break;
    }
  }

  //This is the method that controls the armExtensionMotor
  public void armExtension(double speed) {

    armExtensionMotor.set(speed);

    if(armExtensionEncoder.getPosition() == middleNodeDistance) {
      redController.setRumble(RumbleType.kLeftRumble, 1);
    }
    else if (armExtensionEncoder.getPosition() == highNodeDistance){
      redController.setRumble(RumbleType.kBothRumble, 1);
    }
  }

  //Clamp Action
  public void bite(Boolean status) {
    if (status) { //Closed with game peice
      clamp.set(Value.kForward);
      teeth.set(0.75);
    } else { //Open
      clamp.set(Value.kReverse);
      teeth.set(0);
    }       
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    //Sets up the drive train. Left stick controls the forward and back. Right controls turning.
    //Want cubic function. currently linear. Look up deadbands
    driveTrain.arcadeDrive(blueController.getRawAxis(1), blueController.getRawAxis(4));

    //need control for exstention motor

    //This bunch of if then statements is the button map. Blue controller is operator
    if (redController.getRawButton(0)) { // Button ✖️. Scoring position
      armLevel(2);
    } else if (redController.getRawButton(1)) { // Button ⭕. Intake level
      armLevel(1);
    } else if (redController.getRawButton(2)) { // Button 🟪. Starting configeration
      armLevel(0);
    } else if (redController.getRawButton(3)) { // Button 🔺. No purpose at the moment.
      
    } else if (redController.getRawButton(4)) { // Button L1. Intake is open
      intakeStatus = false;
    } else if (redController.getRawButton(5)) { // Button R1. Intake is closed
      intakeStatus = true;
    } else if (redController.getRawButton(6)) { // Button SHARE.

    }

    armExtension(redController.getRawAxis(1));

    bite(intakeStatus);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
