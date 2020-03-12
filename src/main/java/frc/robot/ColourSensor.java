/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class ColourSensor extends TimedRobot {
  Motor motor;
  I2C.Port port;
  ColorSensorV3 ColourSensor;
  ColorMatch ColourMatcher;
  Color blueTriangle;
  Color greenTriangle;
  Color redTriangle;
  Color yellowTriangle;

  String totColour;
  String initColour;
  String colourString = "";
  String lastColour;
  int coloursSeen = 0;

  boolean doSpin;
  boolean turn;
  boolean readyToSpin;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  public ColourSensor(boolean isPracticeBot) {
    motor = new Motor(15, "ColourWheel", 4096, 0.1, 0, 0, 0.31);
    port = I2C.Port.kOnboard;
    ColourSensor = new ColorSensorV3(port);
    ColourMatcher = new ColorMatch();
    blueTriangle = ColorMatch.makeColor(0.152, 0.441, 0.407);
    greenTriangle = ColorMatch.makeColor(0.190, 0.555, 0.255);
    redTriangle = ColorMatch.makeColor(0.430, 0.392, 0.174);
    yellowTriangle = ColorMatch.makeColor(0.319, 0.538, 0.141);

    totColour = "";
    initColour = "default";
    colourString = "default";
    lastColour = "Blue";
    coloursSeen = 0;

    doSpin = false;
    turn = false;
    readyToSpin = false;
    //ColourSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes20bit,
    // 1, 0x00); // 0x00 = 20bit resolution
    ColourMatcher.addColorMatch(blueTriangle);
    ColourMatcher.addColorMatch(greenTriangle);
    ColourMatcher.addColorMatch(redTriangle);
    ColourMatcher.addColorMatch(yellowTriangle);
    SmartDashboard.putNumber("Red:", 0);
    SmartDashboard.putNumber("Green:", 0);
    SmartDashboard.putNumber("Blue:", 0);
    SmartDashboard.putNumber("Confidence:", 0);
    SmartDashboard.putString("Detected Colour:", "Unknown");
    SmartDashboard.putString("Last Colour: ", "who knows");
    SmartDashboard.putNumber("Colour Wheel RPM: ", (motor.countsToUnits(motor.motor.getSelectedSensorVelocity())));
    motor.motor.setInverted(false);
    motor.motor.setNeutralMode(NeutralMode.Brake);
    totColour = "";
    coloursSeen = 0;
  }

  public void startSpin() {
    totColour = "";
    lastColour = colourString; // when start pressed, run once
    doSpin = true;
  }

  public void startTurn() {
    turn = true;
  }

  public void stopSpin() {
    motor.setVelocity(0);
    coloursSeen = 0;
    doSpin = false;
    turn = false;
  }

  private String currentColour() {
    final Color detectedColour = ColourSensor.getColor();
    final ColorMatchResult match = ColourMatcher.matchClosestColor(detectedColour);

    if (match.color == blueTriangle) {
      colourString = "Blue";
    } else if (match.color == redTriangle) {
      colourString = "Red";
    } else if (match.color == greenTriangle) {
      colourString = "Green";
    } else if (match.color == yellowTriangle) {
      colourString = "Yellow";
    } else {
      colourString = "Unknown";
    }
    SmartDashboard.putNumber("Red:", detectedColour.red);
    SmartDashboard.putNumber("Green:", detectedColour.green);
    SmartDashboard.putNumber("Blue:", detectedColour.blue);
    SmartDashboard.putNumber("Confidence:", match.confidence);
    SmartDashboard.putNumber("ColoursSeen: ", coloursSeen);
    SmartDashboard.putString("Detected Colour:", colourString);
    SmartDashboard.putString("Last Colour: ", lastColour);
    SmartDashboard.putString("Total Colour: ", totColour);
    SmartDashboard.putNumber("Colour Wheel RPM: ",
        (motor.countsToUnits(motor.motor.getSelectedSensorVelocity())) * 600.0);
    return colourString;
  }

  public void spinTimes() {
    if (doSpin) {
      colourString = currentColour();

      if (lastColour.equals("Blue") && colourString.equals("Green")) {
        totColour += "B ";
        lastColour = "Green";
        coloursSeen++;
      }
      if (lastColour.equals("Green") && colourString.equals("Red")) {
        totColour += "G ";
        lastColour = "Red";
        coloursSeen++;
      }
      if (lastColour.equals("Red") && colourString.equals("Yellow")) {
        totColour += "R ";
        lastColour = "Yellow";
        coloursSeen++;
      }
      if (lastColour.equals("Yellow") && colourString.equals("Blue")) {
        totColour += "Y ";
        lastColour = "Blue";
        coloursSeen++;
      }

      if (coloursSeen < 24) {
        motor.setVelocity(200);
      } else {
        motor.setVelocity(0);
      }
    }
  }

  public void turnTo() {
    if (turn) {
      colourString = currentColour();
      final String neededColour = DriverStation.getInstance().getGameSpecificMessage();
      SmartDashboard.putString("Needed Colour: ", neededColour);

      if (neededColour.equals("R")) {
        if (!colourString.equals("Blue")) {
          motor.setVelocity(100);
        } else {
          motor.setVelocity(0);
        }
      } else if (neededColour.equals("Y")) {
        if (!colourString.equals("Green")) {
          motor.setVelocity(100);
        } else {
          motor.setVelocity(0);
        }
      } else if (neededColour.equals("B")) {
        if (!colourString.equals("Red")) {
          motor.setVelocity(100);
        } else {
          motor.setVelocity(0);
        }
      } else if (neededColour.equals("G")) {
        if (!colourString.equals("Yellow")) {
          motor.setVelocity(100);
        } else {
          motor.setVelocity(0);
        }
      }
    }
  }

  public void setPIDF() {
    motor.getPIDF();
  }

  public void doThings() {
    spinTimes();
    turnTo();
  }
}