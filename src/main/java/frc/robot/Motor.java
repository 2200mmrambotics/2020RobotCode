package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Motor {
    public TalonSRX motor;
    public double defaultp;
    public double defaulti;
    public double defaultd;
    public double defaultf;
    public double p;
    public double i;
    public double d;
    public double f;
    public String name;
    double encoderCountsPerUnit;
    double positionSetPoint;
    double velocitySetPoint;

    // Calls Motor with extra steps
    public Motor(int port, String name, double encoderCountsPerUnit) {
        this(port, name, encoderCountsPerUnit, 0, 0, 0, 0);
    }

    /*
     * Creates an object that allows for TalonSRX use that makes some parts easier
     */
    public Motor(int port, String name, double encoderCountsPerUnit, double pV, double iV, double dV, double fV) {
        motor = new TalonSRX(port);
        this.name = name;
        p = pV;
        i = iV;
        d = dV;
        f = fV;
        defaultp = p;
        defaulti = i;
        defaultd = d;
        defaultf = f;
        setPIDF(p, i, d, f);
        positionSetPoint = 0;
        velocitySetPoint = 0;
        this.encoderCountsPerUnit = encoderCountsPerUnit;

        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        
    }

    // Change the encoder counts needed per unit of choice
    public void setEncoderCountsPerUnit(double input) {
        encoderCountsPerUnit = input;
    }

    public void setDefaultPIDF() {
        setPIDF(defaultp, defaulti, defaultd, defaultf);
    }

    // Set PID for the motor to tune the settings
    public void setPIDF(double pV, double iV, double dV, double fV) {
        p = pV;
        i = iV;
        d = dV;
        f = fV;
        motor.config_kP(0, p);
        motor.config_kI(0, i);
        motor.config_kD(0, d);
        motor.config_kF(0, f);
    }

    // For position control
    public void setPosition(double position) {
        positionSetPoint = unitsToCounts(position);
        motor.set(ControlMode.Position, positionSetPoint);
    }

    // Move at speed
    public void setVelocity(double velocity) {
        velocitySetPoint = unitsToCounts(velocity) / 600.0;
        motor.set(ControlMode.Velocity, velocitySetPoint);
    }

    // Check to see if velocity is within a margin of error
    public boolean isAtVelocitySetPoint(double marginOfError) {
        return isNumberinRange(countsToUnits(velocitySetPoint) * 600.0,
                countsToUnits(motor.getSelectedSensorVelocity()) * 600.0, marginOfError);
    }

    // Check to see if position is within a margin of error
    public boolean isAtPositionSetPoint(double marginOfError) {
        return isNumberinRange(positionSetPoint, countsToUnits(motor.getSelectedSensorPosition()), marginOfError);
    }

    // Check to see if a number is within a range
    private boolean isNumberinRange(double needle, double haystack, double range) {
        return (((haystack - range) < needle) // greater than low bounds
                && (needle < (haystack + range))); // less than upper bounds
    }

    // set a motor to output a certain percent of power
    public void setPercentOutput(double value) {
        motor.set(ControlMode.PercentOutput, value);
    }

    // Converts input units to encoder counts
    public double unitsToCounts(double units) {
        return units * encoderCountsPerUnit;
    }

    // Turns encoder value into a specific unit
    public double countsToUnits(double counts) {
        return counts / encoderCountsPerUnit;
    }

    // Puts PID values to smartdashboard
    public void printPIDF() {
        SmartDashboard.putNumber("P " + name, p);
        SmartDashboard.putNumber("I " + name, i);
        SmartDashboard.putNumber("D " + name, d);
        SmartDashboard.putNumber("F " + name, f);
    }

    // Grabs PID from smartdashboard
    public void getPIDF() {
        p = SmartDashboard.getNumber("P " + name, p);
        i = SmartDashboard.getNumber("I " + name, i);
        d = SmartDashboard.getNumber("D " + name, d);
        f = SmartDashboard.getNumber("F " + name, f);
        setPIDF(p, i, d, f);
    }
}