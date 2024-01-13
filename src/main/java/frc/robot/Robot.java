// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Limit switches stop the robot from damaging itself.
  public final DigitalInput limitLiftTop = new DigitalInput(9);
  public final DigitalInput limitLiftBottom = new DigitalInput(8);

  // Enable controls from both our left and right joysticks.
  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);

  // These variables initialize/set the motors for use.
  private final PWMSparkMax leftWheel = new PWMSparkMax(Constants.LEFTWHEEL);
  private final PWMSparkMax rightWheel = new PWMSparkMax(Constants.RIGHTWHEEL);
  private final DifferentialDrive robotDrive = new DifferentialDrive(leftWheel, rightWheel);

  // These are variables for our left and right wheel encoders. Using values set
  // in 'Constants.java'.
  private final Encoder leftWheelEncoder = new Encoder(Constants.LEFTMOTORENCODERONE, Constants.LEFTMOTORENCODERTWO,
      false, EncodingType.k2X);
  private final Encoder rightWheelEncoder = new Encoder(Constants.RIGHTMOTORENCODERONE, Constants.RIGHTMOTORENCODERTWO,
      false, EncodingType.k2X);

  private final PWMSparkMax liftMotor = new PWMSparkMax(Constants.liftMotor);
  private final PWMSparkMax grabberMotor = new PWMSparkMax(Constants.grabberMotor);

  private static boolean liftingAuto;
  private static double targetLiftStopTime;
  /*
   * These variables are for our bot's auto mode.
   * 
   * 'step' is our current instruction, after each instruction is finished this
   * value increments.
   * 'autoTime' assists in storing how many ticks/seconds had passed after each
   * instruction, 'stepTime' does this as well.
   * 'gripperAutoMove' tells telop to keep gripper moving until trigger is pressed.
   */
  private static int step = 0;
  private static double timeLastTargetUpdated = 0;
  private static double autoTime = 0;
  private static double stepTime = 0;

  // Below section is both our wheels current distance.
  private double leftWheelEncoderDist = 0;
  private double rightWheelEncoderDist = 0;
  private double curveLeftMult = 1f;

  // This is our target distance for our wheels, mainly for 'DriveDistance'
  // function and auto mode.
  private double leftWheelTargetDist = 0;
  private double rightWheelTargetDist = 0;

  // Mainly for 'distanceTick' function. If we're driving by distance, if these
  // are true, we stop the motors controlling the wheels.
  private boolean leftIsGoingBackwards = true;
  private boolean rightIsGoingBackwards = true;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    double dpt = -2 * Constants.MOTORAXELRADIUS * Math.PI / Constants.MOTORENCODERTPR;
    leftWheelEncoder.setDistancePerPulse(dpt); // todo
    rightWheelEncoder.setDistancePerPulse(-dpt); // todo
    DataLogManager.start();
    //UsbCamera camera = CameraServer.startAutomaticCapture();
    grabberMotor.setInverted(true);
    liftMotor.setInverted(true);
    //camera.setResolution(240, 180);

    leftWheelEncoder.reset();
    rightWheelEncoder.reset();
    leftWheelEncoderDist = leftWheelEncoder.getDistance();
    rightWheelEncoderDist = rightWheelEncoder.getDistance();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    autoTime += getPeriod();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    step = 0;
    autoTime = 0;
    leftWheelEncoder.reset();
    rightWheelEncoder.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    DistanceTick();
    SmartDashboard.putNumber("Left Target Distance", leftWheelTargetDist);
    SmartDashboard.putNumber("Right Target Distance", rightWheelTargetDist);
    SmartDashboard.putNumber("Current Step", step);
    switch (step) {
      case 0:
        if (leftWheelTargetDist == 0 && rightWheelTargetDist == 0) {
          DriveDistance(-FootToMillimeter(0.5), -FootToMillimeter(0.5), 0.95d);
        } else {
          step = 1;
        }
        break;
      case 1:
        if (leftWheelTargetDist == 0 && rightWheelTargetDist == 0)
          step = 2;
        break;
      case 2:
        liftingAuto = true;
        targetLiftStopTime = autoTime + Constants.LIFT_TIME;
        liftMotor.set(Constants.REVERSE_LIFT ? -0.25 : 0.25d);
        if (leftWheelTargetDist == 0 && rightWheelTargetDist == 0) {
          DriveDistance(-FootToMillimeter(Constants.MOVE_DIST_FT), -FootToMillimeter(Constants.MOVE_DIST_FT), 0.95d);
        } else {
          step = 3;
        }
        break;
      case 3:
        if (autoTime >= targetLiftStopTime || !limitLiftTop.get())
          liftMotor.set(0d);
        else
        {
          double speed = 0.25d + Math.min(1d, (autoTime - (targetLiftStopTime - Constants.LIFT_TIME)));
          liftMotor.set(Constants.REVERSE_LIFT ? -speed : speed);
        }
          
        if (leftWheelTargetDist == 0 && rightWheelTargetDist == 0)
          step = 4;
        break;
      case 4:
        break;
      // case 0:
      //   // XXX: (Remove after testing) Should come to an abrupt stop and bring the gripper down, modify if otherwise.
      //   stepTime = autoTime;
      //   step = 1;
      //   DriveDistance(800.1, 800.1);
      //   break;
      // case 1:
      //   if (leftWheelTargetDist == 0 && rightWheelTargetDist == 0) {
      //     step = 2;
      //     stepTime = autoTime;
      //     DriveDistance((15 * Math.PI) / 4 * 25.4, -(15 * Math.PI) / 4 * 25.4);
      //     break;
      //   }
      //   break;
      // case 2:
      //   if (leftWheelTargetDist == 0 && rightWheelTargetDist == 0 && !Lift.isCalibrating) {
      //     step = 3;
      //     stepTime = autoTime;
      //     Lift.shouldMoveUp = true;
      //   }
      //   break;
      // case 3:
      //   if (Lift.getPosition() >= 16) {
      //     Lift.shouldMoveUp = false;
      //     step = 4;
      //     stepTime = autoTime;
      //   }
      //   break;
      // case 4:
      //   step = 5;
      //   stepTime = autoTime;
      //   break;
      // case 5:
      //   if (autoTime - stepTime >= 5) {
      //     step = 6;
      //     stepTime = autoTime;
      //   }
      //   break;

      // case 6:
      //   gripper.set(-0.5);
      //   gripperAutoMove = true;
      //   step = 7;
      //   stepTime = autoTime;
      //   break;
      // case 7:
      //   DriveDistance(-90, -90);
      //   break;

    }
  }

  // Basically 'DriveDistance' extended. Runs each tick.
  public void DistanceTick() {
    leftWheelEncoderDist = leftWheelEncoder.getDistance();
    rightWheelEncoderDist = rightWheelEncoder.getDistance();
    SmartDashboard.putBoolean("l bkwd", leftIsGoingBackwards);
    SmartDashboard.putBoolean("r bkwd", rightIsGoingBackwards);
    SmartDashboard.putNumber("l encoder", leftWheelEncoderDist);
    SmartDashboard.putNumber("r encoder", rightWheelEncoderDist);
    SmartDashboard.putNumber("l target", leftWheelTargetDist);
    SmartDashboard.putNumber("r target", rightWheelTargetDist);
    double left = 0d, right = 0d;
    if (leftIsGoingBackwards) {
      if (leftWheelEncoderDist < leftWheelTargetDist) {
        leftWheelTargetDist = 0;
      }
      else if (leftWheelTargetDist != 0)
        left = -Constants.MOTORSPEED;
    } else {
      if (leftWheelEncoderDist > leftWheelTargetDist) {
        leftWheelTargetDist = 0;
      }
      else if (leftWheelTargetDist != 0)
        left = Constants.MOTORSPEED;
    }
    if (rightIsGoingBackwards) {
      if (rightWheelEncoderDist < rightWheelTargetDist) {
        rightWheelTargetDist = 0;
      }
      else if (rightWheelTargetDist != 0)
        right = -Constants.MOTORSPEED;
    } else {
      if (rightWheelEncoderDist > rightWheelTargetDist) {
        rightWheelTargetDist = 0;
      }
      else if (rightWheelTargetDist != 0)
        right = Constants.MOTORSPEED;
    }
    if (rightWheelTargetDist == 0 && leftWheelTargetDist == 0)
      curveLeftMult = 1;
    SmartDashboard.putNumber("left", left);
    SmartDashboard.putNumber("curve", curveLeftMult);
    SmartDashboard.putNumber("right", right);
    if (this.isAutonomousEnabled())
      Drive(left * curveLeftMult, right / curveLeftMult);
  }

  /*
   * Instead of driving a set amount, this function drives by distance wanted.
   * 
   * Once that distance is reached, both motors stops.
   */

  public double FootToMillimeter(double v) {
    return v * 12 * 25.4;
  }
  public void DriveDistance(double left, double right, double curveLeft) {
    if (left < 0) {
      //leftspeed *= -1;
      leftIsGoingBackwards = true;
    } else {
      leftIsGoingBackwards = false;
    }
    if (right < 0) {
      //rightspeed *= -1;
      rightIsGoingBackwards = true;
    } else {
      rightIsGoingBackwards = false;
    }
    curveLeftMult = curveLeft <= 0 ? 1 : curveLeft;
    leftWheelTargetDist = leftWheelEncoder.getDistance() + left;
    rightWheelTargetDist = rightWheelEncoder.getDistance() + right;
    timeLastTargetUpdated = autoTime;
  }

  // Drives each wheel a set amount using the left and right parameters.
  public void Drive(double left, double right) {
    robotDrive.tankDrive(-left * Constants.LEFTBIAS, right / Constants.LEFTBIAS, false);
  }

  /** This function is called once when teleop is enabled. */
  @Override 
  public void teleopInit() {
    autoTime = 0;
    step = 0;
    leftWheelEncoder.reset();
    rightWheelEncoder.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    DistanceTick();

    // Stores our current Y axis from both joysticks.
    double leftYAxis = leftJoystick.getY();
    double leftXAxis = leftJoystick.getX();
    double leftZAxis = (-leftJoystick.getZ()+1)/2;

    robotDrive.arcadeDrive(leftXAxis*leftZAxis, -leftYAxis*leftZAxis);
    
    if (leftJoystick.getRawButton(3) && limitLiftTop.get()) {
      liftMotor.set(Constants.REVERSE_LIFT ? -1 : 1); 
    }
    else if (leftJoystick.getRawButton(2) && limitLiftBottom.get()) {
      liftMotor.set(Constants.REVERSE_LIFT ? 1 : -1);
    }
    else {
      liftMotor.set(0);
    }

    if (leftJoystick.getRawButton(1)) {
      grabberMotor.set(0.3);
      SmartDashboard.putBoolean("Last Opened", false);
    }
    else if (rightJoystick.getRawButton(1)) {
      grabberMotor.set(-0.3);
      SmartDashboard.putBoolean("Last Opened", true);
    }
    else {
      grabberMotor.set(0); 
    }
    // Drive(leftYAxis * -leftZAxis * -1 / Constants.SMOOTHDURATION, rightYAxis * -rightZAxis * -1 / Constants.SMOOTHDURATION);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    double leftJoystick_dir = leftJoystick.getY();
    double rightJoystick_dir = rightJoystick.getY();



    SmartDashboard.putNumber("Left Joystick", leftJoystick_dir);
    SmartDashboard.putNumber("Right Joystick", rightJoystick_dir);

  }
}
