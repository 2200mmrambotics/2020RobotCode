/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Robot Objects
  Climb climb;
  Intake intake;
  Drive drive;
  Shooter shooter;
  ColourSensor colourSensor;
  Compressor compressor;

  // Controllers
  XboxController driver;
  XboxController codriver;

  // Timers
  Timer timer;

  // Auto Selectors and steps
  String autoSelected;
  SendableChooser<String> chooser;
  int autostep;

  boolean isPracticeBot = true;
  boolean target = false;
  boolean abortMission = false;

  // auto programs
  // 1 = 4/5 ball left - tested - works
  // 2 = 8 ball right - untested
  // 3 = 6 ball right - tested - works
  // 4 = 9 ball right team pass
  // 5 = 3 drive back
  // 6 = Test Under Trench
  int autoProgram = 6;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();
    timer = new Timer();
    chooser = new SendableChooser<String>();
    chooser.setDefaultOption("8BallRight", "8BallRight");
    chooser.addOption("10BallLeft", "10BallLeft");
    chooser.addOption("6BallRight", "6BallRight");
    chooser.addOption("9BallRightTeamPass", "9BallRightTeamPass");
    SmartDashboard.putData("auto mode", chooser);

    climb = new Climb(isPracticeBot);
    driver = new XboxController(1);
    codriver = new XboxController(0);
    intake = new Intake(isPracticeBot);
    drive = new Drive(isPracticeBot);
    shooter = new Shooter(isPracticeBot, drive.navx);
    colourSensor = new ColourSensor(isPracticeBot);
    shooter.printPIDF();
    compressor = new Compressor(30);
    drive.navx.zeroYaw();
    drive.leftFrontDrive.setSmartCurrentLimit(50);
    drive.leftRearDrive.setSmartCurrentLimit(50);
    drive.rightFrontDrive.setSmartCurrentLimit(50);
    drive.rightRearDrive.setSmartCurrentLimit(50);
    drive.leftFrontDrive.burnFlash();
    drive.leftRearDrive.burnFlash();
    drive.rightFrontDrive.burnFlash();
    drive.rightRearDrive.burnFlash();

    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void robotPeriodic() {
    drive.printTemperature();
    drive.printSpeed();
    drive.printPosition();

    /*
     * else { SmartDashboard.putNumber("Pan Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Tilt Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Top Wheel Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Bottom Wheel Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Middle Fondler Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Left Fondler Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Right Fondler Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Left Front Neo Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Left Rear Neo Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Right Front Neo Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Right Rear Neo Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Intake Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Left Climb Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Right Climb Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Colour Wheel Current: ", pdp.getCurrent(0));
     * SmartDashboard.putNumber("Limelight Current: ", );
     * 
     * 
     * 
     * }
     */

    SmartDashboard.putBoolean("NavX Connected", drive.navx.isConnected());
    SmartDashboard.putNumber("Navx Angle: ", drive.navx.getAngle());
    SmartDashboard.putNumber("Autostep: ", autostep);
    SmartDashboard.putBoolean("Target?", true);
  }

  // Take an input, scale it to size, shift correct amount. Returns clamped value
  public static double scaleAndClamp(final double input, final double inLow, final double inHigh, final double outLow,
      final double outHigh) {

    final double m = (outHigh - outLow) / (inHigh - inLow);
    final double b = outLow - inLow * m;
    double output = m * input + b;
    if (output > outHigh) {
      output = outHigh;
    }
    if (output < outLow) {
      output = outLow;
    }
    return output;
  }

  @Override
  public void autonomousInit() {
    // Grab the wanted autonomous for use
    autoSelected = chooser.getSelected();
    autostep = 0;

    // Prepare drive for automatic use
    drive.resetEncoders();
    drive.navx.zeroYaw();
    drive.leftFrontDrive.setIdleMode(IdleMode.kBrake);
    drive.leftRearDrive.setIdleMode(IdleMode.kBrake);
    drive.rightFrontDrive.setIdleMode(IdleMode.kBrake);
    drive.rightRearDrive.setIdleMode(IdleMode.kBrake);

    // Prepare timer for use
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    double panAdjust = 0;
    double tiltAdjust = 0;
    final double driveSpeed = drive.speed();
    if (!drive.navx.isConnected()) {
      abortMission = true;
    }

    if (!abortMission) {
      if (autoProgram == 1) { // 4/5 ball left
        if (autostep == 0) { // Prepare intake to grab balls and drive for 2.7 metres
          intake.down();
          intake.in();
          shooter.disableActiveAim();
          shooter.enableTrenchTilt();
          if (drive.driveDistance(2.7, 10, 0) < 0.1) {
            autostep = 6;
          }
        }
        if (autostep == 6) { // Prepare drive for next distance travelled
          drive.resetEncoders();
          autostep++;
        }
        if (autostep == 7) { // drive backwards 0.5 metres
          if (drive.driveDistance(-1, -6, 0) > -0.1) {
            autostep++;
          }
        }
        if (autostep == 8) { // Prepare drive for next distance travelled
          drive.resetEncoders();
          autostep++;
        }
        if (autostep == 9) { // Stop intake and drive backwards 3.2 metres
          intake.stop();
          if (drive.driveDistance(-3.7, -10, 0) > -0.1) {
            autostep++;
          }
        }
        if (autostep == 10) { // Turn 90 degrees to the left
          if (drive.turnDegrees(-90, 6.0) > -22) {
            autostep++;
            shooter.disableTrenchTilt();
          }
        }
        if (autostep == 11) { // Prepare drive for next distance travelled
          drive.resetEncoders();
          autostep++;
        }
        if (autostep == 12) { // Prepare and fire shooter with current balls. Drive 0.5 metres forward
          shooter.table.getEntry("pipeline").setNumber(8);
          panAdjust = -45.0;
          // tiltAdjust = 1.0;
          shooter.enableActiveAim();
          /*
           * if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
           * shooter.enableShootWhenReady(); }
           */
          if (drive.driveDistance(3.0, 10, -90) < 0.1) {
            autostep++;
          }
        }
        if (autostep == 13) { // Prepare drive for next distance travelled
          drive.resetEncoders();
          panAdjust = -45;
          if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
            shooter.enableShootWhenReady();
          }
          intake.up();
          intake.stop();
          autostep = 90;
        }

        panAdjust = shooter.angleToControllerPercent(panAdjust);
        shooter.doThings(panAdjust, tiltAdjust, driveSpeed); // Control shooter
      } else if (autoProgram == 2) { // 8 ball right
        if (autostep == 0) {
          shooter.table.getEntry("pipeline").setNumber(8);
          timer.reset();
          timer.start();
          autostep++;
        }
        if (autostep == 1) {
          intake.down();
          intake.in();
          shooter.enableActiveAim();
          panAdjust = 150;
          if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
            shooter.enableShootWhenReady();
          }
          if (drive.driveDistance(2, 7.5, 0) < 0.1) {
            autostep++;
            shooter.disableActiveAim();
            shooter.disableShootWhenReady();
          }
        }
        if (autostep == 2) {
          // drive.resetEncoders();
          autostep++;
        }
        if (autostep == 3) {

          shooter.enableTrenchTilt();
          if (drive.driveDistance(6, 10, 0) < 0.1) {
            autostep++;
            drive.resetEncoders();
          }
        }
        if (autostep == 4) {
          drive.resetEncoders();
          autostep++;
        }
        if (autostep == 5) {
          if (drive.driveDistance(-2, -8, 0) > -0.1) {
            autostep++;
          }
        }
        if (autostep == 6) {
          drive.resetEncoders();
          autostep++;
          shooter.disableTrenchTilt();
        }
        if (autostep == 7) {
          intake.stop();
          if (drive.driveDistance(-3, -12, 0) > -0.1) {
            autostep++;
          }
        }
        if (autostep == 8) {
          if (drive.turnDegrees(-120, -10, 0.4) > -20) {
            autostep++;
          }
        }
        if (autostep == 9) {
          shooter.enableActiveAim();
          intake.in();
          panAdjust = -70;
          if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
            shooter.enableShootWhenReady();
          }

        }

        panAdjust = shooter.angleToControllerPercent(panAdjust);
        shooter.doThings(panAdjust, tiltAdjust, driveSpeed); // Control the shooter
      } else if (autoProgram == 3) { // 6 ball right
        if (autostep == 0) {
          shooter.table.getEntry("pipeline").setNumber(8);
          timer.reset();
          timer.start();
          autostep++;
        }
        if (autostep == 1) {
          intake.down();
          intake.in();
          panAdjust = 160;
          shooter.enablePrePan();
          if (timer.get() > 0.5) {
            autostep++;
          }
        }
        if (autostep == 2) {
          panAdjust = 160;
          shooter.disablePrePan();
          shooter.enableActiveAim();
          if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
            shooter.enableShootWhenReady();
            autostep++;
          }
        }
        if (autostep == 3) {

          if (drive.driveDistance(2, 7.5, 0) < 0.1) {
            autostep++;
            shooter.disableActiveAim();
            shooter.disableShootWhenReady();
          }
        }
        if (autostep == 4) {

          shooter.enableTrenchTilt();
          if (drive.driveDistance(4.2, 10, 0) < 0.1) {
            autostep++;
            drive.resetEncoders();
          }
        }
        if (autostep == 5) {
          drive.resetEncoders();
          autostep++;
        }
        if (autostep == 6) {
          if (drive.driveDistance(-2.2, -8, 0) > -0.1) {
            autostep++;
          }
        }
        if (autostep == 7) {
          drive.resetEncoders();
          autostep++;
          shooter.disableTrenchTilt();
        }
        if (autostep == 8) {
          intake.stop();
          if (drive.driveDistance(-1, -12, 0) > -0.1) {
            autostep++;
            timer.reset();
            timer.start();
          }
        }
        if (autostep == 9) {
          panAdjust = 160;
          shooter.enablePrePan();
          intake.in();
          if (timer.get() > 1) {
            autostep++;
          }
        }
        if (autostep == 10) {
          panAdjust = 160;
          shooter.disablePrePan();
          shooter.enableActiveAim();
          if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
            shooter.enableShootWhenReady();
          }
          if (timer.get() > 4) {
            autostep++;
          }
        }
        if (autostep == 11) {
          shooter.disableShootWhenReady();
          shooter.disableActiveAim();
          intake.stop();
          intake.up();
          panAdjust = 0;
        }
        panAdjust = shooter.angleToControllerPercent(panAdjust);
        shooter.doThings(panAdjust, tiltAdjust, driveSpeed);
      } else if (autoProgram == 4) { // 9 ball partner pass
        if (autostep == 0) {
          shooter.table.getEntry("pipeline").setNumber(8);
          intake.down();
          intake.in();
          drive.resetEncoders();
          shooter.enableActiveAim();
          panAdjust = 60;
          if (shooter.table.getEntry("tv").getDouble(0.0) > 0 && timer.get() > 0.5) {
            shooter.enableShootWhenReady();
          }
          if (timer.get() > 5) {
            autostep++;
            shooter.disableActiveAim();
            shooter.disableShootWhenReady();
            intake.stop();
          }

        }
        if (autostep == 1) {
          if (drive.driveDistance(-1, -9, 0) > -0.1) {
            autostep++;
          }
        }

        if (autostep == 2) {
          if (drive.turnDegrees(-45, 6, 0.8) > -20)
            autostep++;
          drive.resetEncoders();
          intake.down();
          intake.in();
        }
        if (autostep == 3) {
          if (drive.turnDegrees(-90, -6, 0.8) > -20)
            autostep++;
          drive.resetEncoders();
        }
        if (autostep == 4) {
          if (drive.driveDistance(5.0, 11, -90) < 0.1) {
            autostep++;
            intake.inSlow();
            drive.resetEncoders();
          }
        }
        if (autostep == 5) {
          if (drive.driveDistance(-1.0, -9, -90) > -0.1) {
            intake.up();
            intake.stop();
            autostep++;
          }
        }
        if (autostep == 6) {
          if (drive.driveDistance(-3.5, -12, -90) > -0.1) {
            autostep = 8;
          }
        }
        if (autostep == 7) {
          if (drive.turnDegrees(-210, -12, 0.4) > -20) {
            autostep++;
            timer.reset();
          }
        }
        if (autostep == 8) {
          intake.in();
          panAdjust = 170;
          if (timer.get() < 0.5) {
            shooter.enablePrePan();
          }
          if (timer.get() > 0.5) {
            shooter.disablePrePan();
            shooter.enableActiveAim();
            if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
              shooter.enableShootWhenReady();
            }
          }

        }

        panAdjust = shooter.angleToControllerPercent(panAdjust);
        shooter.doThings(panAdjust, tiltAdjust, driveSpeed);
      } else if (autoProgram == 5) { // 3 ball drive back
        if (autostep == 0) {
          timer.reset();
          timer.start();
          autostep++;
        }
        if (autostep == 1) {
          shooter.table.getEntry("pipeline").setNumber(8);
          intake.down();
          shooter.enableActiveAim();
          if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
            shooter.enableShootWhenReady();
          }
          if (timer.get() > 4) {
            shooter.disableActiveAim();
            shooter.disableShootWhenReady();
            autostep++;
            drive.resetEncoders();
          }
        }
        if (autostep == 2) {
          if (drive.driveDistance(-1, -8, 0) > -0.1) {
            autostep++;
          }
        }
        panAdjust = shooter.angleToControllerPercent(panAdjust);
        shooter.doThings(panAdjust, tiltAdjust, driveSpeed);
      } else if (autoProgram == 6) {
        if (autostep == 0) {
          shooter.table.getEntry("pipeline").setNumber(8);
          timer.reset();
          timer.start();
          drive.resetEncoders();
          intake.down();
          intake.in();
          autostep++;
        }
        if (autostep == 1) {
          shooter.enablePrePan();
          panAdjust = 175;
          if (timer.get() > 0.6) {
            autostep++;
          } 
        }
        if (autostep == 2) {
          shooter.disablePrePan();
          shooter.enableActiveAim();
          panAdjust = 150;
          if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
            shooter.enableShootWhenReady();
          }
          if (drive.driveDistance(2.5, 8, 0) < 0.1) {
            autostep++;
            shooter.disableActiveAim();
            shooter.disableShootWhenReady();
            shooter.enableTrenchTilt();
          }
        }
        if (autostep == 3) {
          if (drive.driveDistance(5.40, 9.5, 0) < 0.1) {
            autostep++;
          }
        }
        if (autostep == 4) {
          if (drive.turnDegrees(10, 6, 0.55) < 3) {
            autostep++;
          }
        }
        if (autostep == 5) {
          drive.resetEncoders();
          autostep++;
        }
        if (autostep == 6) {
          if (drive.driveDistance(0.32, 10, 10) < 0.1) {
            autostep++;
          }
        }
        if (autostep == 7) {
          drive.resetEncoders();
          autostep++;
        }
        if (autostep == 8) {
          if (drive.driveDistance(-0.6, -10, 10) > -0.1) {
            autostep++;
          }
        }
        if (autostep == 9) {
          if (drive.turnDegrees(-10, 6, 0.55) > -3) {
            autostep++;
          }
        }
        if (autostep == 10) {
          drive.resetEncoders();
          autostep++;
        }
        if (autostep == 11) {
          if (drive.driveDistance(0.25, 10, -10) < 0.1) {
            autostep++;
          }
        }
        if (autostep == 12) {
          if (drive.turnDegrees(0, -10, 0.55) < 3) {
            autostep++;
          }
        }
        if (autostep == 13) {
          drive.resetEncoders();
          autostep++;
        }
        if (autostep == 14) {
          if (drive.driveDistance(-3.98, -12, 0) > -0.1) {
            autostep++;
            shooter.disableTrenchTilt();
          }
        }
        if (autostep == 15) {
          shooter.enablePrePan();
          panAdjust = 110;
          if (drive.turnDegrees(40, 10, 0.75) < 20) {
            autostep++;
            drive.resetEncoders();
          }
        }
        if (autostep == 16) {
          panAdjust = 110;
          shooter.disablePrePan();
          shooter.enableActiveAim();
          if (drive.driveDistance(1, 11, 60) < 0.1) {
            autostep++;
          }
        }
        if(autostep == 17){
          panAdjust = 110;
          if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
            shooter.enableShootWhenReady();
          }
          if (drive.driveDistance(2., 7, 65) < 0.1) {
            autostep++;
          }
        }
        if(autostep == 18){
          panAdjust = 110;
          if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
            shooter.enableShootWhenReady();
          }
          if (drive.driveDistance(2.25, 5, 65) < 0.1) { // slow drive to pick up balls
            autostep++;
          }
        }
        panAdjust = shooter.angleToControllerPercent(panAdjust);
        shooter.doThings(panAdjust, tiltAdjust, driveSpeed);
      }
    }
  }

  @Override
  public void teleopInit() {
    drive.leftFrontDrive.setIdleMode(IdleMode.kCoast);
    drive.leftRearDrive.setIdleMode(IdleMode.kCoast);
    drive.rightFrontDrive.setIdleMode(IdleMode.kCoast);
    drive.rightRearDrive.setIdleMode(IdleMode.kCoast);

    shooter.disablePrePan();
    shooter.disableActiveAim();
  }

  @Override
  public void teleopPeriodic() {
    // Check for a limelight target. prints true to the dashboard if found
    if (shooter.table.getEntry("tv").getDouble(0.0) > 0) {
      target = true;
      codriver.setRumble(RumbleType.kLeftRumble, 1);
      codriver.setRumble(RumbleType.kRightRumble, 1);
    } else {
      target = false;
      codriver.setRumble(RumbleType.kLeftRumble, 0);
      codriver.setRumble(RumbleType.kRightRumble, 0);
    }

    // Intake Functions
    if (driver.getAButtonPressed()) {
      intake.down();
      intake.in();
    } else if (driver.getAButtonReleased()) {
      intake.stop();
    }

    if (driver.getBButtonPressed()) {
      intake.out();
    } else if (driver.getBButtonReleased()) {
      intake.stop();
    }

    if (driver.getYButtonPressed()) {
      intake.up();
    }

    // Prepare robot for trench run
    if (driver.getXButton()) {
      shooter.enableTrenchTilt();
    } else {
      shooter.disableTrenchTilt();
    }

    // unjam fondlers
    if (codriver.getYButtonPressed()) {
      shooter.unjam(true);
    }
    if (codriver.getYButtonReleased()) {
      shooter.unjam(false);
    }

    // Code only available before using the climber
    if (climb.stowed) {

      // Start the active aim of the ball shooter
      if (codriver.getTriggerAxis(Hand.kLeft) > 0.8) {

        // Control pipeline used by limelight for tracking
        if (codriver.getBumper(Hand.kLeft)) {
          shooter.table.getEntry("pipeline").setNumber(9);
          shooter.threePtPanAdjustCoeff = shooter.highResThreePtPanAdjustCoeff;
        } else {
          shooter.table.getEntry("pipeline").setNumber(8);
          shooter.threePtPanAdjustCoeff = shooter.defaultThreePtPanAdjustCoeff;
        }

        // Prepare for automated scoring
        shooter.enableActiveAim();
        shooter.disableShootOverride();
        if (codriver.getTriggerAxis(Hand.kRight) > 0.8) {
          shooter.enableShootWhenReady();
        } else {
          shooter.disableShootWhenReady();
        }
      } else {

        // Start shooting regardless of current aim
        shooter.disableActiveAim();
        if (codriver.getTriggerAxis(Hand.kRight) > 0.8) {
          shooter.enableShootOverride();
        } else {
          shooter.disableShootWhenReady();
          shooter.disableShootOverride();
        }
      }

      // Set PID values from dashboard. Remove once tuned
      if (codriver.getBButtonPressed()) {
        shooter.setPIDF();
        colourSensor.setPIDF();
      }

      // Get the shooter in position to spin the colour wheel
      if (codriver.getPOV() == 90) {
        shooter.tiltToSpin();
        drive.changeCutoffOrRange(false); // Enable Scaling of drive
        drive.setForwardLimit(0.3);
        drive.setTurnLimit(0.3);
      }

      // Return robot to normal operation
      if (codriver.getPOV() == 270) {
        shooter.untiltFromSpin();
        drive.changeCutoffOrRange(true); // Enable Cutoff of drive
        drive.setForwardLimit(1.0);
        drive.setTurnLimit(0.55);
      }
    }

    // Prepare climb arms for safe release
    if (driver.getBumper(Hand.kLeft) && driver.getBumper(Hand.kRight) && driver.getTriggerAxis(Hand.kLeft) > 0.8
        && driver.getTriggerAxis(Hand.kRight) > 0.8) {
      shooter.disableActiveAim();
      shooter.disableShootOverride();
      shooter.disableShootWhenReady();
      shooter.stowForever();
      intake.down();
    }

    // Release arms
    if (driver.getStartButton() && driver.getBackButton() && shooter.stowed) {
      climb.releaseArms();
    }

    // Prepare to use the colour wheel for a specific task
    if (codriver.getStartButtonPressed()) {
      colourSensor.startSpin();
    } else if (codriver.getBackButtonPressed()) {
      colourSensor.startTurn();
    }

    // Stop all spinning of the colour wheel
    if (codriver.getBackButtonReleased() || codriver.getStartButtonReleased()) {
      colourSensor.stopSpin();
    }

    // Grab input from codriver to manualy control turret until target is found
    final double codriverRightHandX = codriver.getX(Hand.kRight);
    final double codriverRightHandY = -1 * codriver.getY(Hand.kRight);

    // Limit speed upwards of climb arms

    // Grab input from driver to move the robot
    double y = driver.getY(Hand.kLeft) * -1;
    double x = driver.getX(Hand.kLeft);

    // Add a deadband
    if (y < 0.08 && y > -0.08) {
      y = 0;
    }

    if (x < 0.08 && x > -0.08) {
      x = 0;
    }

    // Manually control the gear shift
    if (driver.getBumper(Hand.kRight)) {
      drive.shiftGear(true);
    } else {
      drive.shiftGear(false);
    }

    // Do drive tasks
    drive.curveDrive(y, x);

    // Allow objects to be run
    climb.doThings(driver.getPOV());
    shooter.doThings(codriverRightHandX, codriverRightHandY, drive.speed());
    colourSensor.doThings();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {

    // Allow for ease of control of solenoids in pits
    if (driver.getAButtonPressed()) {
      climb.armRelease.set(Value.kReverse);
    } else if (driver.getAButtonReleased()) {
      climb.armRelease.set(Value.kForward);
    }

    if (driver.getXButtonPressed()) {
      intake.down();
    } else if (driver.getYButtonPressed()) {
      intake.up();
    }
  }

  @Override
  public void disabledInit() {
    shooter.table.getEntry("ledMode").setNumber(1);
    drive.leftFrontDrive.setIdleMode(IdleMode.kCoast);
    drive.leftRearDrive.setIdleMode(IdleMode.kCoast);
    drive.rightFrontDrive.setIdleMode(IdleMode.kCoast);
    drive.rightRearDrive.setIdleMode(IdleMode.kCoast);
  }

}
