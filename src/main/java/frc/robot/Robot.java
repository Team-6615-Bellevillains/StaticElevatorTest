// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;

public class Robot extends TimedRobot {
    private WPI_TalonSRX verticalElevatorMotorA;
    private WPI_TalonSRX verticalElevatorMotorB;
    private Encoder verticalElevatorEncoder;
    private final CommandXboxController xboxController = new CommandXboxController(0);
    private final int kSlowFallVoltage = 2;

    @Override
    public void robotInit() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);

                verticalElevatorMotorA = new WPI_TalonSRX(ElevatorConstants.verticalMotorAPort);
                verticalElevatorMotorB = new WPI_TalonSRX(ElevatorConstants.verticalMotorBPort);

                verticalElevatorMotorA.setInverted(true);

                verticalElevatorEncoder = new Encoder(0, 1);
                verticalElevatorEncoder.setSamplesToAverage(15);
                verticalElevatorEncoder.setReverseDirection(true);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }).start();


    }

    private static final double tickRate = 50;

    private double voltage = 0;
    private double voltageIncreasePerTick = .1 / tickRate;
    private double kGPlus = 0;
    private double kGMinus = 0;

    private double kG = 0;
    private double kS = 0;

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {
        verticalElevatorEncoder.setSamplesToAverage(15);
    }

    @Override
    public void autonomousPeriodic() {
        System.out.printf("Curr %s volts", voltage);

        SmartDashboard.putNumber("Timer", Timer.getFPGATimestamp());
        SmartDashboard.putNumber("Rate", verticalElevatorEncoder.getRate());

        SmartDashboard.putNumber("Voltage", voltage);
        SmartDashboard.putNumber("A Stator Current", verticalElevatorMotorA.getStatorCurrent());
        SmartDashboard.putNumber("A Supply Current", verticalElevatorMotorA.getSupplyCurrent());
        SmartDashboard.putNumber("B Stator Current", verticalElevatorMotorB.getStatorCurrent());
        SmartDashboard.putNumber("B Supply Current", verticalElevatorMotorB.getSupplyCurrent());
        if (verticalElevatorEncoder.getRate() > 2 && voltageIncreasePerTick >= 0) {
            SmartDashboard.putNumber("kG+", voltage);
            kGPlus = voltage;
            voltageIncreasePerTick *= -1;
        } else if (kGPlus != 0 && verticalElevatorEncoder.getRate() <= 0) {
            if (kGMinus == 0) {
                kGMinus = voltage;
                SmartDashboard.putNumber("kG-", voltage);
                voltageIncreasePerTick = 0;

                SmartDashboard.putNumber("kG", (kGPlus + kGMinus) / 2);
                SmartDashboard.putNumber("kS", Math.abs(kGPlus - kGMinus) / 2);
            }
        }

        if (voltageIncreasePerTick == 0) {
            lowerOrStopElevator();
        } else {
            voltage += voltageIncreasePerTick;
            verticalElevatorMotorA.setVoltage(voltage);
            verticalElevatorMotorB.setVoltage(voltage);
        }
    }


    private ProfiledPIDController profiledPIDController = new ProfiledPIDController(0.05, 0, 0, new TrapezoidProfile.Constraints(.10, 0.05));
    private ElevatorFeedforward elevatorFeedforward;

    @Override
    public void teleopInit() {
        verticalElevatorEncoder.setSamplesToAverage(3);

        elevatorFeedforward = new ElevatorFeedforward(kS, kG, 0.147613 * Units.inchesToMeters(1));
        profiledPIDController.setGoal(Units.inchesToMeters(15));
        profiledPIDController.reset(getVerticalEncoderDistance());
    }

    public double getVerticalEncoderDistance() {
        return verticalElevatorEncoder.get() * (ElevatorConstants.verticalRotationsToDistance / ElevatorConstants.verticalEncoderPulsesPerRevolution);
    }

    public void lowerOrStopElevator() {
        if (verticalElevatorEncoder.get() != 0) {
            voltage = kSlowFallVoltage;
        } else {
            voltage = 0;
        }

        verticalElevatorMotorA.setVoltage(voltage);
        verticalElevatorMotorB.setVoltage(voltage);
    }

    @Override
    public void teleopPeriodic() {
        if (xboxController.a().getAsBoolean()) {
            SmartDashboard.putNumber("Position", getVerticalEncoderDistance());
            double pidOutput = profiledPIDController.calculate(getVerticalEncoderDistance());
            double feedforward = elevatorFeedforward.calculate(profiledPIDController.getSetpoint().velocity);

            verticalElevatorMotorA.setVoltage(pidOutput + feedforward);
            verticalElevatorMotorB.setVoltage(pidOutput + feedforward);
        } else if (xboxController.b().getAsBoolean()){
            verticalElevatorMotorA.setVoltage(elevatorFeedforward.calculate(0));
            verticalElevatorMotorB.setVoltage(elevatorFeedforward.calculate(0));
        } else {
            lowerOrStopElevator();
        }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        if (verticalElevatorMotorA != null) {
            lowerOrStopElevator();
        }
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
