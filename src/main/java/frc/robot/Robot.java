/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import javax.swing.plaf.IconUIResource;

//import com.ctre.phoenix.sensors.PigeonIMU;

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

  //using to calculate distance from pp
  private final double a1 = 15.5*Math.PI/180; //Angle of the camera to the ground
  private double a2;
  private final double deltaHeight = 80; //Limelight to the target
  private double visionDistance; // from the limelight to the base of the target


  //Origin to Power Port
  private final double OPP = 2743.0/12; //inches, equal to 228 7/12 inches as a mixed fraction
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

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void testInit() {

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

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    robotHeading = 0; //set as a gyro value;
    currentRotation = new Rotation2d(robotHeading); // making it the robotheading

    if (limelight.getEntry("tv").getDouble(0) == 1) {

      tx = limelight.getEntry("tx").getDouble(0);
      ty = limelight.getEntry("ty").getDouble(0);
      visionDistance = deltaHeight/(Math.tan(a1+a2));
      turretHeading = robotHeading + rAngle;
      turretHeading %= 2*Math.PI;//one rotation

      if(turretHeading<Math.PI) {
        calcX = visionDistance*Math.sin(turretHeading);
        calcY = ppY-visionDistance*Math.cos(turretHeading);
      } else {
        calcX = lenX-visionDistance*Math.sin(turretHeading);
        calcY = lenY-ppY+visionDistance*Math.cos(turretHeading-Math.PI);
      }

      currentLocation = new Pose2d(calcX,calcY,currentRotation);

      //reset odometry with the new data

    }else{
      currentLocation = moOdometry.update(currentRotation,leftEncoderDistance,rightEncoderDistance);

    }

    SmartDashboard.putNumber("Vision Distance", visionDistance);
//    SmartDashboard.putNumber("RobotX", calcX);
//    SmartDashboard.putNumber("RobotY", calcY);
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