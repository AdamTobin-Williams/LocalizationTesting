/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team2485.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team2485.WarlordsLib.robotConfigs.RobotConfigs;

private static final String kDefaultAuto = "Default";
private static final String kCustomAuto = "My Auto";
private String m_autoSelected;
private final SendableChooser<String> m_chooser = new SendableChooser<>();

private Pose2d currentLocation;
private Rotation2d currentRotation;

//using to calculate distance from pp
private final double a1 = 15.5*Math.PI/180; //Angle of the camera to the ground in 1 of our tests(not on actual robot)
private double a2;
private final double deltaHeight = 80; //Limelight to the target
private double visionDistance; // from the limelight to the base of the target


//Origin to Power Port
private final double OPP = 2743.0/12; //inches, equal to 228 7/12 inches
private final double lenX = 629.25; //inches
private final double lenY = 323.25; //inches
private double calcX, calcY;

//odometry
private DifferentialDriveOdometry moOdometry;
private double leftEncoderDistance;
private double rightEncoderDistance;

//limelight other values
private NetworkTable limelight;
private double tx;
private double ty;

//angles of the robot and turret to the field
private double robotHeading;
private double turretHeading;
private final double ppY = OPP;
//Angle difference from the robots heading and the turrets heading, negative if it turned left and positive if it turned right, can be >360 and <-360
private double rAngle;
//Robots current heading/rotation in relation to the field in Degrees and Radians
private double currRotDeg;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    RobotConfigs.getInstance().loadConfigsFromFile(Constants.CONFIGS_FILE);
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    RobotConfigs.getInstance().saveConfigsToFile(Constants.CONFIGS_FILE);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    localizationInit();
  }

  @Override
  public void autonomousPeriodic() {
    Pose2d pos = localizationPeriodic();
    SmartDashboard.putNumber("Robot Heading", pos.getRotation().getDegrees);
    SmartDashboard.putNumber("Robot X", pos.getTranslation().getX());
    SmartDashboard.putNumber("Robot Y", pos.getTranslation().getY());
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.resetAll();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
  public void localizationInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    currentRotation = new Rotation2d(-Math.PI / 2);
    currentLocation = new Pose2d(154.875/*Inches*/, 0, currentRotation);
    limelight = NetworkTableInstance.getDefault().getTable("limelight");


    //random values
    ty = 0;
    a2 = ty;
    visionDistance=0;
    rAngle = 0;
    robotHeading = 0;
    turretHeading = 0;
    tx = 0;
    leftEncoderDistance = 0;
    rightEncoderDistance = 0;

    currentRotation = new Rotation2d(robotHeading); // making it the robotheading
    //Making robot odometry
    moOdometry = new DifferentialDriveOdometry(currentRotation,currentLocation);
  }
  public Pose2d localizationPeriodic() {

    robotHeading = 0; //set as a gyro value;
    currentRotation = new Rotation2d(robotHeading); // making it the robotheading

    if (limelight.getEntry("tv").getDouble(0) == 1) {

      tx = limelight.getEntry("tx").getDouble(0);
      ty = limelight.getEntry("ty").getDouble(0);
      visionDistance = deltaHeight/(Math.tan(a1+a2));
      turretHeading = robotHeading + rAngle;
      turretHeading %= 2*Math.PI;//one rotation

      if(turretHeading<Math.PI) {
        calcX = visionDistance*Math.sin(turretHeading+tx);
        calcY = ppY-visionDistance*Math.cos(turretHeading+tx);
      } else {
        calcX = lenX-visionDistance*Math.sin(turretHeading-Math.PI+tx);
        calcY = lenY-ppY+visionDistance*Math.cos(turretHeading-Math.PI+tx);
      }

      currentLocation = new Pose2d(calcX,calcY,currentRotation);

      //reset odometry with the new data
      moOdometry.resetPosition(currentLocation,currentRotation);
    }else{
      currentLocation = moOdometry.update(currentRotation,Drivetrain.m_encoderLeft.getPosition(),Drivetrain.m_encoderRight.getPosition());
    }
    SmartDashboard.putNumber("Vision Distance", visionDistance);
//    SmartDashboard.putNumber("RobotX", calcX);
//    SmartDashboard.putNumber("RobotY", calcY);
    return currentLocation;
  }
}
