/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.networktables.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Pose2d currentLocation;
  private Rotation2d currentRotation;
  private double currX, currY;
  private double camtranAngle;
  //PowerPort
  private double distanceFromPP;
  //Origin to Power Port
  private final double OPP = 252; //inches, probably wrong
  private final double lenX = 629.25; //inches
  private final double lenY = 323.25; //inches
  private double[] camtran;
  private double calcX, calcY;

  //Angle difference from the robots heading and the turrets heading, negative if it turned left and positive if it turned right, can be >360 and <-360
  private double rAngle;
  //Robots current heading/rotation in relation to the field in Degrees and Radians
  private double currRotRad, currRotDeg;

  // Tx value from limelight
  private double tx;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    currentRotation = new Rotation2d(-Math.PI / 2);
    currentLocation = new Pose2d(154.875/*Inches*/, 0, currentRotation);
    camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getBoolean(false)) {
      camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
      camtranAngle = camtran[4];
      distanceFromPP = camtran[0];
      //find the placement of the robot on the field by using the angle given by camtran and the distance from the target
      calcX = distanceFromPP * Math.cos(camtranAngle);
      calcY = distanceFromPP * Math.sin(camtranAngle);
      currRotDeg = 90+ camtranAngle /*Yaw*/ - rAngle - tx;
      //Stephen: the best loser
      if (currRotDeg+rAngle+tx <= 0) {
        currY = OPP + calcY;
        currX = calcX;
        currRotDeg+=180;
      } else {
        currY = lenY - OPP - calcY;
        currX = lenX-calcX;
      }

      //Find the Rotation of the robot in relation to the field
      //Reduce the rotation to within one half rotation

      //Do Current Rotation logic
      currRotDeg %= 360;//one rotation
      if (currRotDeg > 180) currRotDeg -= 360; //one half rotation if more than 180
      currentRotation = new Rotation2d(currRotDeg * Math.PI / 180.0);

      currentLocation = new Pose2d(currX, currY, currentRotation);
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
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

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
