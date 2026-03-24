// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
    private static final int CLIMBER_CAN_ID = 16;
    private static final int CURRENT_LIMIT_AMPS = 40;
    private static final boolean INVERTED = false;

    private final SparkMax climberMotor = new SparkMax(CLIMBER_CAN_ID, MotorType.kBrushless);

    @SuppressWarnings("removal")
    public LiftSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(CURRENT_LIMIT_AMPS)
            .inverted(INVERTED);

        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climbUp (double speed) {
        climberMotor.set(speed);
        System.out.println("Climber Running at : " + speed);
    }

    public void stop() {
        climberMotor.set(0.0);
        System.out.println("Climber Stopped");
    }
}