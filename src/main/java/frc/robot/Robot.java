// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;

public class Robot extends TimedRobot {
    private WPI_TalonSRX verticalElevatorMotorA;
    private WPI_TalonSRX verticalElevatorMotorB;
    private Encoder verticalElevatorEncoder;

    @Override
    public void robotInit() {
        verticalElevatorMotorA = new WPI_TalonSRX(ElevatorConstants.verticalMotorAPort);
        verticalElevatorMotorB = new WPI_TalonSRX(ElevatorConstants.verticalMotorBPort);

        verticalElevatorMotorA.setInverted(true);

        verticalElevatorEncoder = new Encoder(0, 1);
        verticalElevatorEncoder.setReverseDirection(true);
    }

    private static final double tickRate = 50;

    private double voltage = 0;
    private double voltageIncreasePerTick = .25 / tickRate;
    private double kGPlus = 0;

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Voltage", voltage);
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
        if (Math.abs(verticalElevatorEncoder.getRate()) > 5) {
            if (voltageIncreasePerTick >= 0) {
                SmartDashboard.putNumber("kG+", voltage);
                kGPlus = voltage;
                voltageIncreasePerTick *= -1;
            } else {
                SmartDashboard.putNumber("kG-", voltage);
                double kGMinus = voltage;
                voltage = 0;
                voltageIncreasePerTick = 0;

                SmartDashboard.putNumber("kG", (kGPlus + kGMinus) / 2);
                SmartDashboard.putNumber("kS", Math.abs(kGPlus - kGMinus) / 2);
            }
        }

        voltage += voltageIncreasePerTick;
        verticalElevatorMotorA.setVoltage(voltage);
        verticalElevatorMotorB.setVoltage(voltage);
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        verticalElevatorMotorA.setVoltage(0);
        verticalElevatorMotorB.setVoltage(0);
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
