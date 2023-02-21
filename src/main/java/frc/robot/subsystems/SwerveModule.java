package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
     // 初始化 rotor & throttle 馬達
    private CANSparkMax mRotor;
    private CANSparkMax mThrottle;

    // 初始化 throttle encoder
    private RelativeEncoder mThrottleEncoder;

    // 初始化 rotor encoder
    private WPI_CANCoder mRotorEncoder;

    // 初始化 rotor PID controller
   // private PIDController mRotorPID;
    private PIDController mRotorPID;


    /**
     * 構建新的 SwerveModule
     *
     * @param throttleID CAN ID of throttle 馬達
     * @param rotorID CAN ID of rotor 馬達
     * @param rotorEncoderID CAN ID of rotor encoder
     * @param rotorOffsetAngleDeg rotor encoder 偏移量
     */
    public SwerveModule(int throttleID, int rotorID, int rotorEncoderID, double rotorOffsetAngleDeg, boolean kRotorEncoderDirection, boolean kThrottleReversed)
    {
        // 實例化 throttle 馬達 & encoder
        mThrottle = new CANSparkMax(throttleID, MotorType.kBrushless);
        mThrottleEncoder = mThrottle.getEncoder();

        // 實例化 rotor 馬達
        mRotor = new CANSparkMax(rotorID, MotorType.kBrushless);

        // 實例化 rotor absolute encoder
        mRotorEncoder = new WPI_CANCoder(rotorEncoderID);

        // 重置所有配置（保險起見以免有舊的配置）
        mThrottle.restoreFactoryDefaults();
        mRotor.restoreFactoryDefaults();
        mRotorEncoder.configFactoryDefault();

        mThrottle.setInverted(kThrottleReversed);

        // 根據之前的常數配置 rotor 馬達
        mRotor.setInverted(SwerveConstants.kRotorMotorInversion);
        mRotor.enableVoltageCompensation(Constants.kVoltageCompensation);
        mRotor.setIdleMode(IdleMode.kCoast);

        // 根據之前的常數配置轉向 rotor encoder
        mRotorEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        mRotorEncoder.configMagnetOffset(rotorOffsetAngleDeg);
        mRotorEncoder.configSensorDirection(kRotorEncoderDirection);
        mRotorEncoder.configSensorInitializationStrategy(
             SensorInitializationStrategy.BootToAbsolutePosition
         );

        

        // 根據之前的常數配置 rotor 馬達的PID控制器
        // mRotorPID = mRotor.getPIDController();
        // mRotorPID.setFF(SwerveConstants.kRotor_kF, 0);
        // mRotorPID.setP(SwerveConstants.kRotor_kP, 0);
        // mRotorPID.setI(SwerveConstants.kRotor_kI, 0);
        // mRotorPID.setD(SwerveConstants.kRotor_kD, 0);

        mRotorPID = new PIDController(
            
            SwerveConstants.kRotor_kP,
            SwerveConstants.kRotor_kI,
            SwerveConstants.kRotor_kD
        );
        mRotorPID.setTolerance(45/ 4096.0 * 360.0);
      
        // ContinuousInput 認為 min 和 max 是同一點並且自動計算到設定點的最短路線
         mRotorPID.enableContinuousInput(-180, 180);
         
      
       

        // 根據之前的常數配置 throttle 馬達
        mThrottle.enableVoltageCompensation(Constants.kVoltageCompensation);
        mThrottle.setIdleMode(IdleMode.kCoast);

        // 給與 throttle encoder 轉換係數以便它以米每秒而不是 RPM 為單位讀取速度
        mThrottleEncoder.setVelocityConversionFactor(
            SwerveConstants.kThrottleVelocityConversionFactor
        );
    }

    /**
     * Return current state of module
     * 
     * @return module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            mThrottleEncoder.getVelocity(),
            Rotation2d.fromDegrees(mRotorEncoder.getAbsolutePosition())
        );
    }

    public void initialPoosture(){
       

    }


    /**
     * Set module state
     * 
     * @param state module state 
     */
    public void setState(SwerveModuleState state) {
        // 優化狀態，使轉向馬達不必旋轉超過 90 度來獲得目標的角度
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
        
        // 通過比較目前角度與目標角度來用 PID 控制器計算轉向馬達所需的輸出
        double rotorOutput = mRotorPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        mRotor.set(rotorOutput);
        mThrottle.set(optimizedState.speedMetersPerSecond);
    }

    public double getDriveEncoderPosition() {
        return mThrottleEncoder.getPosition() * SwerveConstants.kThrottleVelocityConversionFactor;
      }

      public double getTurningEncoderAngle() {
        // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * pi) divided by the
        // encoder resolution.
        return mRotorEncoder.getPosition() / 4096.0 * 360.0;
      }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveEncoderPosition(), Rotation2d.fromDegrees(getTurningEncoderAngle()));
    }
}
