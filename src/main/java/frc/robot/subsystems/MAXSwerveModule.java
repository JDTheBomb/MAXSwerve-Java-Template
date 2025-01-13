// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;

import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMaxSim m_drivingSparkSim;

  private final SparkMax m_turningSpark;
  private final SparkMaxSim m_turningSparkSim;

  private final RelativeEncoder m_drivingEncoder;
  private final SparkRelativeEncoderSim m_drivingEncoderSim;

  private final AbsoluteEncoder m_turningEncoder;
  private final SparkAbsoluteEncoderSim m_turningEncoderSim;

  private final SparkClosedLoopController m_drivingClosedLoopController;

  private final SparkClosedLoopController m_turningClosedLoopController;

  private final DCMotor m_drivingGearboxSim = DCMotor.getNEO(1);
  private final DCMotor m_turningGearboxSim = DCMotor.getNeo550(1);

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private final FlywheelSim m_drivingFlywheelSim =
  new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
          m_drivingGearboxSim, 4 * 0.00032, 1), // 0.00032 kg * m^2 is for 1 4"x1.5" Colson
      m_drivingGearboxSim);

  private final FlywheelSim m_turningFlywheelSim =
    new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            m_turningGearboxSim, 4 * 0.00032, 1), // 0.00032 kg * m^2 is for 1 4"x1.5" Colson
        m_turningGearboxSim);

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_drivingSparkSim = new SparkMaxSim(m_drivingSpark, m_drivingGearboxSim);

    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
    m_turningSparkSim = new SparkMaxSim(m_turningSpark, m_turningGearboxSim);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_drivingEncoderSim = m_drivingSparkSim.getRelativeEncoderSim();

    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();
    m_turningEncoderSim = m_turningSparkSim.getAbsoluteEncoderSim();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }



  public double getDrivingSparkAppliedVoltage() {
    //System.out.println("AppliedVoltage:"+m_sparkMax.getAppliedOutput());
    return m_drivingSpark.getAppliedOutput() * RobotController.getInputVoltage();
  }
  public double getturningSparkAppliedVoltage() {
    //System.out.println("AppliedVoltage:"+m_sparkMax.getAppliedOutput());
    return m_turningSpark.getAppliedOutput() * RobotController.getInputVoltage();
  }
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    double timestep = 20e-4;
    m_drivingFlywheelSim.setInputVoltage(getDrivingSparkAppliedVoltage());
    m_drivingFlywheelSim.update(timestep);
    m_drivingSparkSim.iterate(m_drivingFlywheelSim.getAngularVelocityRPM(), RobotController.getInputVoltage(), timestep);

    m_turningFlywheelSim.setInputVoltage(getturningSparkAppliedVoltage());
    m_turningFlywheelSim.update(timestep);
    m_turningSparkSim.iterate(m_turningFlywheelSim.getAngularVelocityRPM(), RobotController.getInputVoltage(), timestep);
    
  }
}
