// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Team 3288 Robot: TBD
 * Programmed by Brandon, Colby, and Mr. N
 */

package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * Base code from WPILib don't change
 **/
public class Robot extends TimedRobot {
  // Sets up autonomous routines.
  private static final String kDefaultAuto = "Default - score_drive_balance";
  private static final String kScoreDriveBackShortAuto = "score_driveBack_short";
  private static final String kScoreDriveBackLongAuto = "score_driveBack_long";
  private static final String KDriveBack = "Driveback";
  private static final String kScoreDriveBackScoreAuto = "score_driveBack_score";
  private static final String kScore = "score";
  private static final String kNothing = "Nothing";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /** Definitions **/
  /* Digital objects connected to physical objects. */
  // The controllers.
  private final GenericHID redController = new GenericHID(1); //Operator Controller
  private final GenericHID blueController = new GenericHID(0); //Drive Controller
  
  // Drive motors
  private final CANSparkMax frontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax backLeftMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final MotorControllerGroup leftDriveMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);

  private final CANSparkMax frontRightMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax backRightMotor = new CANSparkMax(4, MotorType.kBrushless);
  private final MotorControllerGroup rightDriveMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);

  //Drive Motor Encoders
  private final RelativeEncoder frontLeftMotorEncoder = frontLeftMotor.getEncoder(); //Front Left Motor Controller
  private final RelativeEncoder backLeftMotorEncoder = backLeftMotor.getEncoder(); //Back Left Motor Controller
  private final RelativeEncoder frontRightMotorEncoder = frontRightMotor.getEncoder(); //Front Right Motor Controller
  private final RelativeEncoder backRightMotorEncoder = backRightMotor.getEncoder(); //Front Back Motor Controller
  
  //The Drive Train
  private final DifferentialDrive driveTrain = new DifferentialDrive(leftDriveMotors, rightDriveMotors);

  
  //DoubleSolenoids
  private final DoubleSolenoid firstStage = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1); //Small solenoid
  private final DoubleSolenoid secondStage = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9); //Big solenoid
  private final DoubleSolenoid clamp = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15); //Clamping solenoid

  //Motor for extension of the arm.
  private CANSparkMax armExtensionMotor = new CANSparkMax(5,MotorType.kBrushed);
  //Extension Motor Encoder
  private final Encoder armExtensionEncoder = new Encoder(0, 1);


  //Intake motors. JK just one
  private final CANSparkMax teeth = new CANSparkMax(7, MotorType.kBrushed);

  
  //This is the gyro. The yaw aixs is set up robotInit(). the x axis is the yaw axis, y is the roll, and z is the pitch.
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  //Creates IMU axises
  public final IMUAxis pitch = IMUAxis.kZ;
  public final IMUAxis yaw = IMUAxis.kX;
  public final IMUAxis roll = IMUAxis.kY;
  
  
  /* PID Controllers */
  //This is the PID control. //Hard code parameters 
  private final PIDController forwardController = new PIDController(0, 0, 0, 0.1);
  private final PIDController angleController = new PIDController(0, 0, 0, 0.1);
  private final PIDController armExtensionController = new PIDController(0, 0, 0, 0.1);
  

  /* Variables for autonomous */
  //set the distance to travel from out of community to balance
  public final double distanceToBalance = 46; //origanal 102, test 1: 46

  //Angle needed to turn and grab the cone after driving back. 
  public final double angleToGrabCone = 90; //subject to change.
  
  //Angle needed to turn and drive back to drive station after grabbing cone.
  public final double angleToDriveBack = 180; //subject to change.

  //Distance to cone.
  public final double coneDistance = 10; //subject to change.

  //Max arm distance for manual arm control
  public final double maxArmExtensionDistance = 203; //NOTE: TBD

  //Autonomous Length in Seconds
  public final double autonomousLengthSeconds = 15;

  
  /* Other necessary variables */
  //provides the status of the intake. Starts in closed position to hold game piece
  public boolean isClosed;
  public boolean isNotSpinning;
  public boolean isSpinningOut;

  //Distance of middle scoring level. needs to be figured out.
  public final double middleNodeDistance = 108;

  //Distance of the high node. In inches
  public final double highNodeDistance = 200; //Impossible unless thrown.
  
  //Length of robot frame for reference.
  public final double lengthOfRobot = 32; 

  //set the override 
  public boolean isOverride = false;

  //robot distance traveled per complete rotation of drive motors. in inches
  public final double distancePerRotation = 2.23;

  //intake speed
  public final double intakeSpeed = 0.75;//Do we want this at 1?

  //Haft speed
  public boolean halfSpeed = false;

  //sets the distance to travel to get out of community in autonomous on the long side
  public final double distanceOutOfCommunityLong = 200; //Original: 225, test 2: 175, test 3: 90, test 4: 75, test 5: 85 good, 
  //test 6: 90, test: 135 short, test 8: 190, test 9: 200
  //distance out of community plus six inches is 198 in inches. This includes the grid

  //sets the distance to travel to get out of community in autonomous on the short side
  public final double distanceOutOfCommunityShort = 140;  //Original: 156, test 1: 140
  //distance out of the community on the short side plus six inches is 138. This includes the grid

  
  /*
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    //Sets up the options you see for auto on SmartDashboard.
    m_chooser.setDefaultOption("Default - score_drive_balance", kDefaultAuto);
    m_chooser.addOption("score_driveBack_short", kScoreDriveBackShortAuto);
    m_chooser.addOption("score_driveBack_long", kScoreDriveBackLongAuto);
    m_chooser.addOption("Drive_back", KDriveBack);
    m_chooser.addOption("score_driveBack_score", kScoreDriveBackScoreAuto);
    m_chooser.addOption("Score", kScore);
    m_chooser.addOption("Do_Nothing", kNothing);
    SmartDashboard.putData("Auto choices", m_chooser);

    //This is where we change the setInverted properties on the motors.
    frontLeftMotor.setInverted(false); //Values TBD...
    backLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    teeth.setInverted(false);

    //Makes the intake closed at the start of the match
    isClosed = false;
    isNotSpinning = true;
    isSpinningOut = false;

    //Sets up the PID controllers.
    angleController.setTolerance(5);
    angleController.enableContinuousInput(-180, 180);
    armExtensionController.setTolerance(5); 
    forwardController.setTolerance(5); 

    //Initialize the arm extension motor
    armExtensionMotor.setInverted(false);

    //Initalize the arm extension encoder
    armExtensionEncoder.setSamplesToAverage(5);
    armExtensionEncoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 0.5); //Spool diamiter 1 inch
    armExtensionEncoder.setMinRate(1.0);
    
    //Sets the encoders to 0
    frontLeftMotorEncoder.setPosition(0);
    backLeftMotorEncoder.setPosition(0);
    frontRightMotorEncoder.setPosition(0);
    backRightMotorEncoder.setPosition(0);

    //Make sure that it is in starting configuration.
    armLevel(0); 

    //Sets up the Cammera
    CameraServer.startAutomaticCapture();
  }

  /*
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //Reports game time.
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());

    //Reports drive motors' encoders' position and distance.
    SmartDashboard.putNumber("Front Right Motor Encoder Position", frontRightMotorEncoder.getPosition());
    SmartDashboard.putNumber("Front Right Motor Encoder Distance (rotations*2.23)", frontRightMotorEncoder.getPosition() * distancePerRotation);
    SmartDashboard.putNumber("Back Right Motor Encoder Position", backRightMotorEncoder.getPosition());
    SmartDashboard.putNumber("Back Right Motor Encoder Distance (rotations*2.23)", backRightMotorEncoder.getPosition() * distancePerRotation);
    SmartDashboard.putNumber("Front Left Motor Encoder Position", frontLeftMotorEncoder.getPosition());
    SmartDashboard.putNumber("Front Left Motor Encoder Distance (rotations*2.23)", frontLeftMotorEncoder.getPosition() * distancePerRotation);
    SmartDashboard.putNumber("Back Left Motor Encoder Position", backLeftMotorEncoder.getPosition());
    SmartDashboard.putNumber("Back Left Motor Encoder Distance (rotations*2.23)", backLeftMotorEncoder.getPosition() * distancePerRotation);

    //Camara to smartdashboard

    //Reports Gyros' axis angles  
    SmartDashboard.putNumber("The pitch axis of the Gyro (z axis): ", getAngleOfAxis(pitch));
    //SmartDashboard.putNumber("The yaw axis of the gyro (x axis): ", getAngleOfAxis(yaw));
    /********* We're on a Roll **********/

    //Reports Arm Motor Encoder Position //Note: Need to calculate distance at some point.
    SmartDashboard.putNumber("Arm Extension Encoder Position", armExtensionEncoder.getDistance());
  }

  /*
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    //sets timer to zero

    //sets all the positions of the encoders to 0
    frontLeftMotorEncoder.setPosition(0);
    frontRightMotorEncoder.setPosition(0);

    //sets it to brake mode
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kBrake);

    //sets the gyro to zero.
    gyro.setYawAxis(yaw);
  }


  /*** AUTONOMOUS AND TELEOP METHODS ***/ 
  /** CONTROL METHODS USED IN AUTONOMOUS AND TELEOP **/

  /* Method for setting the level of the arm
   * @param int level; 0: starting configuration, 1: ground level, 2: scoring level.
   */
  public void armLevel(int level) {
    switch(level) {//Up and down motion of the arm. Need to figure out how to start with this.
      case 0: //Starting configuration  
       // armExtension(0, false);//retracts the arm
        firstStage.set(Value.kForward);
        secondStage.set(Value.kForward);
        break;

      case 1: //Ground Level Configuration
        //Need to do math here. Put arm to Ground level
        firstStage.set(Value.kReverse);
        secondStage.set(Value.kReverse);
        break;

      case 2: //Scoring Level Configuration
        firstStage.set(Value.kForward);
        secondStage.set(Value.kReverse);
        break;
      
      case 3://just higher than scoring level
        firstStage.set(Value.kReverse);
        secondStage.set(Value.kForward);
        break;

      default:
        break;
    }
  }

  /* This method will return the angle of the wanted axis
   * @param "axis" is the axis you want
   */
  public double getAngleOfAxis(IMUAxis axis) {
    gyro.setYawAxis(axis);
    return gyro.getAngle();
  } 

  /* This method will balance the charging station once called.
   * 
   */
  public void balance(boolean usingGyro) {
    //Puts the robot in brake mode
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
      backLeftMotor.setIdleMode(IdleMode.kBrake);
      frontRightMotor.setIdleMode(IdleMode.kBrake);
      backRightMotor.setIdleMode(IdleMode.kBrake);
    if (usingGyro){
      //balances useing the gyro
      while (getAngleOfAxis(pitch) > 10) {//if it has a positive pitch forward then slowly backs up
        driveTrain.arcadeDrive(-0.25, 0);
      }
      while (getAngleOfAxis(pitch) < -10) {//if it has a negative pitch backward then slowly goes forward
        driveTrain.arcadeDrive(0.25, 0);
      }
      while(getAngleOfAxis(pitch) > -10 && getAngleOfAxis(pitch) < 10){
        driveTrain.arcadeDrive(0, 0);
        bite(false, true, false);
        
        //Could add recursion in this bit. Have a two second delay then run this method agin. 
        //It would have to turminate with time.
        /*Timer.delay(2);
        if(Timer.getMatchTime() > 5){
          balance(true);
        }example*/
      }
    } else {
      driveTrain.arcadeDrive(0, 0);
    }
  }

  /* This is the method that controls the clamp 
   * @param isClosed is the boolean that controls the intake.
   */
  public void bite(boolean isIntakeClosed, boolean isStill, boolean isSpinOut) {
    if (isIntakeClosed) {//intake closed
      clamp.set(Value.kForward);
    } else if(!isIntakeClosed) {//intake open
      clamp.set(Value.kReverse);
    } 
    
    if (isStill) { //Motor are still. Meaning they're not moving
      teeth.set(0);
    } else if (!isSpinOut) { //wheels spinning in
      teeth.set(intakeSpeed);
    } else { //wheels going out
      teeth.set(-intakeSpeed);
    }
  }

  /*This method calculates controller deadzone. 
   * @param axisLevel takes the axis' setpoint.
   * @param controller takes the controller type as a string. Red or Blue.
   * @return float for the deadzone control.
  */
  public double deadzone(double d, String controller) {
    if (controller.equalsIgnoreCase("red")) {
      if ((d > 0.15) || (d < -0.15)) {
        return d;
      }
    }
    if (controller.equalsIgnoreCase("blue")) {
      if ((d > 0.15) || (d < -0.15)) {
        return d;
      }
    }
    return 0;
  }

  /** AUTONOMOUS ONLY METHODS **/
  
  /* This method will be used to score on the middle or high nodes during autonomous.
   * @param boolean isHigh is used to say if it is going high or not.
   */
  public void score(boolean isHigh) {

    //used to score the game peices on certain levels.
    armLevel(3); //arm to scoring level
    
    //Used to get the time at the time the method is called
    double startofMethod = Timer.getFPGATimestamp();

    //Settling delay.
    Timer.delay(1);

    if (isHigh) {
      //need to check the time. While the arm is shorter than the high node distance and time is more than 3 seconds.
      while((armExtensionEncoder.getDistance() < highNodeDistance - 20) /*|| (Timer.getFPGATimestamp() - startofMethod < 1)*/) { 
        armExtensionMotor.set(0.75); //Extends arm at speed of 0.75.
      }
    } else {
      while ((armExtensionEncoder.getDistance() < middleNodeDistance - 20) /*|| (Timer.getFPGATimestamp() - startofMethod < 1)*/) { 
        armExtensionMotor.set(0.75); //Extends arm at speed of 0.75.
      }
    }
    
    //slop delay
    Timer.delay(0.5);
    
    bite(true, false, true);//spews the game piece
    
    Timer.delay(1);

    bite(true, true, false); //stops spew

    //retracts the arm until encoder postion is less than or time is 6sec.
    while ((armExtensionEncoder.getDistance() > 20) /*|| (Timer.getFPGATimestamp() - startofMethod < 6 )*/) { 
      armExtensionMotor.set(-0.75);
    }
  }

  /* Method for returning the encoder that has leased number of rotions.
   * @return returns a relative encoder
   */
  public RelativeEncoder distanceDriven() {
    if (frontLeftMotorEncoder.getPosition() > frontRightMotorEncoder.getPosition()){
      return frontRightMotorEncoder;
    }
      return frontLeftMotorEncoder;
  }

  /* Method for driving a certain distance. Uses the .getPosition() from an encoder to return the number of rotations.
   * @param distance for the distance for the robot to drive.
   */
  public void driveDistance(double distance) {

    //Used to get the time at the time the method is called
    //double startofMethod = Timer.getFPGATimestamp();

    //drives the distance passed in by the parameter out of the community.
    if (distance < 0) {
      while ((frontLeftMotorEncoder.getPosition() * distancePerRotation > distance) /*&& (Timer.getFPGATimestamp() - startofMethod  < 5)*/) { 
        driveTrain.arcadeDrive(-0.6, 0);
      }
    } else {
      while ((frontLeftMotorEncoder.getPosition() * distancePerRotation < distance)/* && (Timer.getFPGATimestamp() - startofMethod  < 5)*/) { 
        driveTrain.arcadeDrive(0.6, 0);
      }
    }
    
  }

  /* Method that is an autonomous rotine. This would be selected in autonomous periodic
   * Would score high and then drive backwards out of the community.
   */
  public void scoreDriveBack(boolean isLong) {

    //this method scores on the different levels. Right now it is used to score high. Ends retracted
    score(true);
    
    if(isLong){
      //drives backward out of the community
      driveDistance(-distanceOutOfCommunityLong);
    } else {
      //drives the short distance out of the community
      driveDistance(-distanceOutOfCommunityShort);
    }
    
  }

  /* Method that is an autonomous routine. This would be selected in autonomous periodic
   * Would score high and then drive backwards out of the community. After that it drives forward to balance
   * on the charging station. 
   */
  public void scoreDriveBackBalance() {
    scoreDriveBack(true);//runs scoreDriveBack

    //drives back to the charging station.
    driveDistance(distanceToBalance - 24);

    //Using the gyro to balance
    balance(false);

    //Waits till the end of auto.
    Timer.delay(autonomousLengthSeconds - Timer.getMatchTime());
  }

  /* Method that is an autonomous routine. This would be selected in autonomous periotic.
   * Will score the cone on the high node and drive backwards out of the community and 
   * then turn to get another cone then turn again drive to the grid and score again.
   * By far this is the most complicated one.
   */
  public void scoreDriveScore() {
    scoreDriveBack(true);

    //turns the robot to the cone 
    while ((getAngleOfAxis(yaw) < 90) && (Timer.getFPGATimestamp() < 5)) {
      driveTrain.arcadeDrive(0, angleController.calculate(getAngleOfAxis(yaw), angleToGrabCone));
    }
    
    //drives into the cone
    driveDistance(coneDistance);
    
    //grabs it
    bite(true, false, false);// opens the intake

    //turns back to driver's station
    while((getAngleOfAxis(yaw) < 135) && (Timer.getFPGATimestamp() < 5)) {
      driveTrain.arcadeDrive(0, angleController.calculate(getAngleOfAxis(yaw), angleToDriveBack));
    }

    //drives back to driver station and scores
    driveDistance(distanceOutOfCommunityLong);
    score(true);
  }

  public void nothing() {
    Timer.delay(autonomousLengthSeconds - Timer.getMatchTime());
  }


  /* This function is called periodically during autonomous.*/
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kScoreDriveBackLongAuto:
        scoreDriveBack(true);
        Timer.delay(Timer.getMatchTime());
        break;

      case kScoreDriveBackShortAuto:
        scoreDriveBack(false);
        Timer.delay(Timer.getMatchTime());
        break;

      case kScoreDriveBackScoreAuto:
        scoreDriveScore();
        //Waits till Auto is over
        Timer.delay(Timer.getMatchTime());
        break;

      case kDefaultAuto:
        scoreDriveBackBalance();
        //Waits till Auto is over
        Timer.delay(Timer.getMatchTime());
        break;

      case kScore:
        armLevel(0);
        //Waits till Auto is over
        Timer.delay(Timer.getMatchTime());
        break;
        
      default:
        scoreDriveBackBalance();
        //Waits till Auto is over
        Timer.delay(Timer.getMatchTime());
        break;
    }
  }

  /* This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //Sets motors IdleMode to coast. Need to add 3 second wait. Figure it out, take your time. Maybe use boolean for brakes.
    frontLeftMotor.setIdleMode(IdleMode.kCoast);
    backLeftMotor.setIdleMode(IdleMode.kCoast);
    frontRightMotor.setIdleMode(IdleMode.kCoast);
    backRightMotor.setIdleMode(IdleMode.kCoast);

    //Make sure that the robot is in full speed.
    halfSpeed = false;
  }

  /* This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    //Sets up the drive train. Left stick controls the forward and back. Right controls turning.
    //Want cubic function. currently linear. Look up deadbands
    if (blueController.getRawButton(5)) { //L1 half speed.
      halfSpeed = true;
    } else if (blueController.getRawButton(6)) {// R1 full speed
      halfSpeed = false;
    }

    if (halfSpeed) { // if half speed is true
      driveTrain.arcadeDrive(-blueController.getRawAxis(1)/2, -blueController.getRawAxis(4)/2);
    } else {
      driveTrain.arcadeDrive(Math.pow(-blueController.getRawAxis(1), 3), Math.pow(-blueController.getRawAxis(4), 3));
    }

    //need control for extention motor. This is the left y axis
    armExtensionMotor.set(Math.pow(-redController.getRawAxis(1), 3));

    //This bunch of if then statements is the button map. Blue controller is operator
    if (redController.getRawButton(1)) { // Button ✖️. Scoring position
      armLevel(2);
    } else if (redController.getRawButton(2)) { // Button ⭕. Intake level
      armLevel(1);
    } else if (redController.getRawButton(3)) { // Button 🟪. Starting configeration
      armLevel(0);
    } else if (redController.getRawButton(4)) {// Button triangle. Just higher than Scoring.
      armLevel(3);
    } else if (redController.getRawButton(5)) { // Button L1. Intake is open
      isClosed = true;
    } else if (redController.getRawButton(6)) { // Button R1. Intake is closed
      isClosed = false;
    } 

    if (redController.getRawAxis(2) > 0.1) { //This sets the variables for the wheels to spin out. Is left trigger (L2)
      isNotSpinning = false;
      isSpinningOut = true;
    } else if (redController.getRawAxis(3) > 0.1) { //This sets the variable for the wheels to spin in. Is right trigger (R2)
      isNotSpinning = false;
      isSpinningOut = false;
    } else { //Wheels not moving.
      isNotSpinning = true;
    }

    //Sets the rumble when the arm is within 0.25 rotation of middle and highNodeDistance.
    if (armExtensionEncoder.getDistance() >= highNodeDistance - 3 && armExtensionEncoder.getDistance() <= highNodeDistance + 3) {
      redController.setRumble(RumbleType.kBothRumble, 0.75);
    } else if (armExtensionEncoder.getDistance() >= middleNodeDistance - 3 && armExtensionEncoder.getDistance() <= middleNodeDistance + 3){
      redController.setRumble(RumbleType.kLeftRumble, 0.75);
    } else {
      redController.setRumble(RumbleType.kBothRumble, 0);
    }

    if (armExtensionEncoder.getDistance() >= middleNodeDistance - 0.25 && armExtensionEncoder.getDistance() <= middleNodeDistance + 0.25) {
      redController.setRumble(RumbleType.kLeftRumble, 1);
    }

    //Set the break mode for drive train
    if (blueController.getRawAxis(2) > 0.1) { //L2 Trigger
      frontLeftMotor.setIdleMode(IdleMode.kBrake);
      backLeftMotor.setIdleMode(IdleMode.kBrake);
      frontRightMotor.setIdleMode(IdleMode.kBrake);
      backRightMotor.setIdleMode(IdleMode.kBrake);
    } else if (blueController.getRawAxis(3) > 0.1) { //R2 Trigger
      frontLeftMotor.setIdleMode(IdleMode.kCoast);
      backLeftMotor.setIdleMode(IdleMode.kCoast);
      frontRightMotor.setIdleMode(IdleMode.kCoast);
      backRightMotor.setIdleMode(IdleMode.kCoast);
    }

    bite(isClosed, isNotSpinning, isSpinningOut);// opens the intake;//controls the intake. false is open, true is closed*/
  }

  /* This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /* This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /* This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /* This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /* This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /* This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
