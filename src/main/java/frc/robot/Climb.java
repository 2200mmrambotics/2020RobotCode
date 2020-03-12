package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb {
    Motor leftClimb;
    Motor rightClimb;
    DoubleSolenoid armRelease;
    Solenoid leftRatchetRelease;
    Solenoid rightRatchetRelease;
    boolean stowed;
    boolean ratcheting;
    boolean upPressed = false;
    double leftMaxHeight = 106;
    double rightMaxHeight = 113;
    Timer timer = new Timer();
    double climbDifference = 0.3;

    public Climb(boolean isPracticeBot) {
        stowed = true;
        ratcheting = true;
        leftClimb = new Motor(9, "LeftClimb", 3500);
        rightClimb = new Motor(10, "RightClimb", 3500);
        if (isPracticeBot) {
            armRelease = new DoubleSolenoid(30, 2, 5);
            leftRatchetRelease = new Solenoid(30, 3);
            rightRatchetRelease = new Solenoid(30, 4);
            leftClimb.motor.setInverted(false);
            rightClimb.motor.setInverted(false);
            leftClimb.motor.setSensorPhase(true);
            rightClimb.motor.setSensorPhase(false);
        } else {
            armRelease = new DoubleSolenoid(30, 5, 0);
            leftRatchetRelease = new Solenoid(30, 6);
            rightRatchetRelease = new Solenoid(30, 7);
            leftClimb.motor.setInverted(true);
            rightClimb.motor.setInverted(true);
            leftClimb.motor.setSensorPhase(false);
            rightClimb.motor.setSensorPhase(true);
        }
        leftClimb.motor.setSelectedSensorPosition(0);
        rightClimb.motor.setSelectedSensorPosition(0);

        

        leftClimb.motor.setNeutralMode(NeutralMode.Coast);
        rightClimb.motor.setNeutralMode(NeutralMode.Coast);

        armRelease.set(Value.kForward);
    }

    public double getLeftArmHeight() {
        return leftClimb.countsToUnits(leftClimb.motor.getSelectedSensorPosition());
    }

    public double getRightArmHeight() {
        return rightClimb.countsToUnits(rightClimb.motor.getSelectedSensorPosition());
    }

    public void ascend() {

    }

    public void ascendLeft() {

    }

    public void ascendRight() {

    }

    public void disengageRatchet() {
        ratcheting = false;
    }

    public void engageRatchet() {
        ratcheting = true;
    }

    public void stop() {
        leftClimb.setPercentOutput(0.0);
        rightClimb.setPercentOutput(0.0);
    }

    public void releaseArms() {
        armRelease.set(Value.kReverse);
        stowed = false;
    }

    public void doThings(int pov) {
        double leftSpeed = 0;
        double rightSpeed = 0;
        double leftPosition = leftClimb.countsToUnits(leftClimb.motor.getSelectedSensorPosition());
        double rightPosition = rightClimb.countsToUnits(rightClimb.motor.getSelectedSensorPosition());
        leftRatchetRelease.set(!ratcheting);
        rightRatchetRelease.set(!ratcheting);
        if (pov == 0) {
            if (!upPressed) {
                ratcheting = false;
                upPressed = true;
                timer.reset();
                timer.start();
                leftSpeed = -0.6;
                rightSpeed = -0.6;
            } else {
                if (timer.get() > 0.15) {
                    double leftArmHeight = getLeftArmHeight();
                    double rightArmHeight = getRightArmHeight();

                    if (leftArmHeight > (rightArmHeight + climbDifference)) {
                        leftSpeed = 0.5;
                        rightSpeed = 0.7;
                    } else if (leftArmHeight < (rightArmHeight - climbDifference)) {
                        leftSpeed = 0.7;
                        rightSpeed = 0.5;
                    } else {
                        leftSpeed = 0.7;
                        rightSpeed = 0.7;
                    }
                } else {
                    leftSpeed = -0.6;
                    rightSpeed = -0.6;
                }
            }
        } else if (pov == 90) {
            ratcheting = true;
            leftSpeed = 0.0;
            rightSpeed = -1.0;
            upPressed = false;
        } else if (pov == 180) {
            ratcheting = true;
            double leftArmHeight = getLeftArmHeight();
            double rightArmHeight = getRightArmHeight();

            if (leftArmHeight > (rightArmHeight + climbDifference)) {
                leftSpeed = -0.9;
                rightSpeed = -0.7;
            } else if (leftArmHeight < (rightArmHeight - climbDifference)) {
                leftSpeed = -0.7;
                rightSpeed = -0.9;
            } else {
                leftSpeed = -0.9;
                rightSpeed = -0.9;
            }
            upPressed = false;
        } else if (pov == 270) {
            ratcheting = true;
            leftSpeed = -1.0;
            rightSpeed = 0.0;
            upPressed = false;
        } else {
            upPressed = false;
            leftSpeed = 0;
            rightSpeed = 0;
        }

        if (leftPosition>leftMaxHeight && pov==0) {
            leftSpeed = 0;
        } else if (leftPosition<0) {
            leftSpeed/=2.0;
        }
        if (rightPosition>rightMaxHeight && pov==0) {
            rightSpeed = 0;
        } else if (rightPosition<0) {
            rightSpeed/=2.0;
        }
        leftClimb.setPercentOutput(leftSpeed);
        rightClimb.setPercentOutput(rightSpeed);
        SmartDashboard.putNumber("Left Climb Position", leftPosition);
        SmartDashboard.putNumber("Right Climb Position", rightPosition);
    }
}