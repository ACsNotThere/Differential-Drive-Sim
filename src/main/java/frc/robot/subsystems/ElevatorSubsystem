// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

/* IMPORTANT: Based on the Reefscape 2025 ElevatorSubsystem code from FRC Team 7902 */

public class ElevatorSubsystem extends SubsystemBase {
    public enum ElevatorPosition {
        CORAL_L1, CORAL_L2, CORAL_L3, CORAL_STATION_AND_PROCESSOR, ALGAE_HIGH, ALGAE_LOW, UNKNOWN
    }
    private final TalonFX m_leaderMotor;
    private final TalonFX m_followerMotor;
    private final TalonFXConfiguration m_motorConfig;
    private final HardwareLimitSwitchConfigs limitConfigs;
    private final ElevatorSim m_elevatorSim;
    private final Mechanism2d m_mech2d;
    private final VoltageOut m_voltReq;
    private final MechanismRoot2d m_mech2dRoot;
    private final SysIdRoutine m_sysIdRoutine;
    private MotionMagicVoltage m_request;
    private Orchestra m_orchestra;
    private MechanismLigament2d m_elevatorMech2d;
    private double m_setpoint;
    private boolean m_homed;
    private String[] m_songs = new String[] {"song1.chrp", "song2.chrp"};

    public ElevatorSubsystem() {
        if (RobotBase.isSimulation()) {
            m_elevatorMech2d.setColor(new Color8Bit(Color.kAntiqueWhite));
        }
        m_leaderMotor = new TalonFX(ElevatorConstants.kElevatorLeaderCAN);
        m_followerMotor = new TalonFX(ElevatorConstants.kElevatorFollowerCAN);
        m_motorConfig = new TalonFXConfiguration();
        limitConfigs = new HardwareLimitSwitchConfigs();
        m_voltReq = new VoltageOut(0.0);
        m_request = new MotionMagicVoltage(0).withSlot(0);
        m_orchestra = new Orchestra();
    
        m_elevatorSim = new ElevatorSim(DCMotor.getFalcon500(2),
                ElevatorConstants.kElevatorGearing, ElevatorConstants.kElevatorCarriageMass,
                ElevatorConstants.kElevatorDrumRadius, ElevatorConstants.kElevatorMinHeightMeters,
                ElevatorConstants.kElevatorMaxHeightMeters, true,
                ElevatorConstants.kElevatorHeightMeters, 0.01, // add some noise
                0);
        
        m_mech2d = new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(50));
        m_mech2dRoot = m_mech2d.getRoot("Elevator Root", Units.inchesToMeters(5), Units.inchesToMeters(0.5));
        m_elevatorMech2d = m_mech2dRoot.append(new MechanismLigament2d("Elevator",
            m_elevatorSim.getPositionMeters(), 90, 7, new Color8Bit(Color.kAntiqueWhite)));
    
        m_sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, Volts.of(4), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism( (volts) -> m_leaderMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
                        null, this));
        m_motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_motorConfig.Slot0.kP = ElevatorConstants.kElevatorP;
        m_motorConfig.Slot0.kI = ElevatorConstants.kElevatorI;
        m_motorConfig.Slot0.kD = ElevatorConstants.kElevatorD;
        m_motorConfig.Slot0.kS = ElevatorConstants.kElevatorS;
        m_motorConfig.Slot0.kV = ElevatorConstants.kElevatorV;
        m_motorConfig.Slot0.kA = ElevatorConstants.kElevatorA;
        m_motorConfig.Slot0.kG = ElevatorConstants.kElevatorG;
        m_motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        m_motorConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kElevatorMaxVelocity;
        m_motorConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorMaxAcceleration;
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 
            ElevatorConstants.kElevatorMaxHeightMeters
            /ElevatorConstants.kElevatorMetersPerMotorRotation;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        m_motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = 125;
        m_motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_motorConfig.CurrentLimits.SupplyCurrentLimit = 60;

        limitConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        limitConfigs.ReverseLimitEnable = true;
        limitConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;

        m_leaderMotor.getConfigurator().apply(limitConfigs);
        m_followerMotor.setControl(new Follower(m_leaderMotor.getDeviceID(), true));
        m_leaderMotor.getConfigurator().apply(m_motorConfig);
        m_followerMotor.getConfigurator().apply(m_motorConfig);

        m_leaderMotor.getPosition().setUpdateFrequency(50);
        m_leaderMotor.getVelocity().setUpdateFrequency(50);
        m_leaderMotor.getDutyCycle().setUpdateFrequency(50);
        m_leaderMotor.getMotorVoltage().setUpdateFrequency(50);
        m_leaderMotor.getTorqueCurrent().setUpdateFrequency(50);
        m_followerMotor.optimizeBusUtilization();
        m_leaderMotor.optimizeBusUtilization();

        m_orchestra.addInstrument(m_leaderMotor);
        m_orchestra.addInstrument(m_followerMotor);

        m_homed = false;
    }

    public double getPosition() {
        return m_leaderMotor.getPosition().getValueAsDouble();
    }
    public double getPositionMeters() {
        return getPosition() * ElevatorConstants.kElevatorMetersPerMotorRotation;
    }
    public void zero() {
        m_leaderMotor.setPosition(0);
    }
    public double getVelocityMetersPerSecond() {
        return m_leaderMotor.getVelocity().getValueAsDouble()
                * ElevatorConstants.kElevatorMetersPerMotorRotation;
    }
    public boolean atHeight() {
        return Math.abs(getPositionMeters() - m_setpoint) < ElevatorConstants.kElevatorTargetError;
    }
    public void stop() {
        m_leaderMotor.stopMotor();
        m_followerMotor.stopMotor();
    }
    public void updateTelemetry() {
        m_elevatorMech2d.setLength(getPositionMeters());
    }

    public void setPosition(double position) {
        if (position > ElevatorConstants.kElevatorMaxHeightMeters) {
            m_setpoint = ElevatorConstants.kElevatorMaxHeightMeters;
        } else if (position < ElevatorConstants.kElevatorMinHeightMeters) {
            m_setpoint = ElevatorConstants.kElevatorMinHeightMeters;
        } else {
            m_setpoint = position;
        }
        double positionRotations = position / ElevatorConstants.kElevatorMetersPerMotorRotation;
        m_request = m_request.withPosition(positionRotations).withSlot(0);
        m_leaderMotor.setControl(m_request);
    }

    public boolean isAtRetractLimit() {
        return m_leaderMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public ElevatorPosition getElevatorEnumPosition() {
        double currentPosition = getPositionMeters();

        if (Math.abs(currentPosition - ElevatorConstants.kElevatorCoralLevel1StartHeight) < 
            ElevatorConstants.kElevatorTargetError * 2) {
            return ElevatorPosition.CORAL_L1;
        } 
        else if (Math.abs(currentPosition - ElevatorConstants.kElevatorCoralLevel2Height) < 
            ElevatorConstants.kElevatorTargetError * 2) {
            return ElevatorPosition.CORAL_L2;
        } 
        else if (Math.abs(currentPosition - ElevatorConstants.kElevatorCoralLevel3Height) < 
            ElevatorConstants.kElevatorTargetError * 2) {
            return ElevatorPosition.CORAL_L3;
        } 
        else if (Math.abs(currentPosition - ElevatorConstants.kElevatorCoralStationAndProcessorHeight) < 
            ElevatorConstants.kElevatorTargetError * 2) {
            return ElevatorPosition.CORAL_STATION_AND_PROCESSOR;
        } 
        else if (Math.abs(currentPosition - ElevatorConstants.kElevatorAlgaeHighHeight) < 
            ElevatorConstants.kElevatorTargetError * 2) {
            return ElevatorPosition.ALGAE_HIGH;
        } 
        else if (Math.abs(currentPosition - ElevatorConstants.kElevatorAlgaeLowHeight) < 
            ElevatorConstants.kElevatorTargetError * 2) {
            return ElevatorPosition.ALGAE_LOW;
        } 
        else {
            return ElevatorPosition.UNKNOWN;
        }
    }

    @Override
    public void periodic() {
        if (m_leaderMotor.getClosedLoopReference().getValueAsDouble() == 0
                && m_leaderMotor.getPosition().getValueAsDouble() < 0.5) {
            m_leaderMotor.setVoltage(0);
        } 
        else {
            m_leaderMotor.setControl(m_request);
        }
        SmartDashboard.putNumber("Elevator position (m)", getPositionMeters());
        SmartDashboard.putNumber("Elevator setpoint position (m)", m_setpoint);
        double metersPerRotationSeconds = (m_leaderMotor.getClosedLoopReference().getValueAsDouble() * 
            ElevatorConstants.kElevatorMetersPerMotorRotation);
        SmartDashboard.putNumber("Elevator velocity (m/s)",
            getVelocityMetersPerSecond());
        SmartDashboard.putNumber("Rotations",
            m_leaderMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Rotations setpoint",
            m_leaderMotor.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Leader supply current",
            m_leaderMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Homed", m_homed);
        SmartDashboard.putNumber("Leader stator current",
            m_leaderMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Follower stator current",
            m_followerMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Follower supply current",
            m_followerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Reverse limit switch", isAtRetractLimit());

        String elevatorEnumPosition = (getElevatorEnumPosition() != null) ? getElevatorEnumPosition().toString()
                 : "N/A";
        SmartDashboard.putString("Curr Position Name", elevatorEnumPosition);

        updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_elevatorSim.setInputVoltage(m_leaderMotor.getMotorVoltage().getValueAsDouble());
        m_elevatorSim.update(0.020);

        final double positionRot = m_elevatorSim.getPositionMeters() / ElevatorConstants.kElevatorMetersPerMotorRotation;
        final double velocityRps = m_elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.kElevatorMetersPerMotorRotation;

        m_leaderMotor.getSimState().setRawRotorPosition(positionRot);
        m_leaderMotor.getSimState().setRotorVelocity(velocityRps);

        // Update battery simulation
        RoboRioSim.setVInVoltage(BatterySim
                .calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
    }

}
