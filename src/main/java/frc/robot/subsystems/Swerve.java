package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
    // Initialize IMU
    // 初始化 IMU
    private final WPI_Pigeon2 mImu = new WPI_Pigeon2(SwerveConstants.kImuID);
    public Runnable drive;
   // private SwerveDriveOdometry mOdometry;
  

    private  SwerveModule mLeftFrontModule = new SwerveModule(
            SwerveConstants.kLeftFrontThrottleID, 
            SwerveConstants.kLeftFrontRotorID, 
            SwerveConstants.kLeftFrontCANCoderID, 
            SwerveConstants.kLeftFrontRotorOffset,
            false, true, false,false
        );

        private  SwerveModule mRightFrontModule = new SwerveModule(
            SwerveConstants.kRightFrontThrottleID, 
            SwerveConstants.kRightFrontRotorID, 
            SwerveConstants.kRightFrontCANCoderID, 
            SwerveConstants.kRightFrontRotorOffset,
            false, true, false,true
        );

        private  SwerveModule mLeftRearModule = new SwerveModule(
            SwerveConstants.kLeftRearThrottleID, 
            SwerveConstants.kLeftRearRotorID, 
            SwerveConstants.kLeftRearCANCoderID, 
            SwerveConstants.kLeftRearRotorOffset,
            false, true, false, false
        );

        private  SwerveModule mRightRearModule = new SwerveModule(
            SwerveConstants.kRightRearThrottleID, 
            SwerveConstants.kRightRearRotorID, 
            SwerveConstants.kRightRearCANCoderID, 
            SwerveConstants.kRightRearRotorOffset,
            false, false, true,false
        );
        //public  SwerveDriveOdometry mOdometry= new SwerveDriveOdometry(SwerveConstants.kSwerveKinematics, mImu.getRotation2d());
    
        private SwerveDriveOdometry mOdometry = new SwerveDriveOdometry(
            SwerveConstants.kSwerveKinematics,  mImu.getRotation2d(),
            new SwerveModulePosition[] {
                mLeftFrontModule.getPosition(),
                mRightFrontModule.getPosition(),
                mLeftRearModule.getPosition(),
                mRightRearModule.getPosition()
            });


        // Instantiate odometry - used for tracking position
        // 實例化（instantiate）測程法（odometry） - 用於追踪位置

        public Swerve() {
            mImu.configFactoryDefault();
            Pigeon2Configuration config = new Pigeon2Configuration();
            config.MountPosePitch = 0;
            config.MountPoseRoll = 0;
            config.MountPoseYaw = 0;
            config.EnableCompass = false;
            config.DisableNoMotionCalibration=false;
            config.DisableTemperatureCompensation=false;
            config.enableOptimizations= false;
        
            mImu.configAllSettings(config);
            mImu.setYaw(0);
          }
        

   
        // Instantiate swerve modules - each、// r、//epresenting unique module on the robot
        // 實例化（instantiate）swerve module - 個代表機器上四個其一

      


         

    @Override
    public void periodic() {
        // Updates odometry with current module state
        // 使用當前Swerve module狀態更新測程法（odometry）。
        mOdometry.update(
            mImu.getRotation2d(), 
            new SwerveModulePosition[] {
                
                mLeftFrontModule.getPosition(),
                mRightFrontModule.getPosition(),//<-
                mLeftRearModule.getPosition(),
                mRightRearModule.getPosition()
            }
        );

        SmartDashboard.putNumber("mLeftFrontModule",  mLeftFrontModule.getDriveEncoderPosition());
        SmartDashboard.putNumber("mRightFrontModule",  mRightFrontModule.getDriveEncoderPosition());
        SmartDashboard.putNumber("mLeftRearModule",  mLeftRearModule.getDriveEncoderPosition());
        SmartDashboard.putNumber("mRightRearModule",  mRightRearModule.getDriveEncoderPosition());

        SmartDashboard.putNumber("AmLeftFrontModule",  mLeftFrontModule.getTurningEncoderAngle());
        SmartDashboard.putNumber("AmRightFrontModule",  mRightFrontModule.getTurningEncoderAngle());
        SmartDashboard.putNumber("AmLeftRearModule",  mLeftRearModule.getTurningEncoderAngle());
        SmartDashboard.putNumber("AmRightRearModule",  mRightRearModule.getTurningEncoderAngle());




    }








    /**
     * Drives the swerve - Input range: [-1, 1]
     * 
     * @param xSpeed percent power in the X direction (X 方向的功率百分比)
     * @param ySpeed percent power in the Y direction (Y 方向的功率百分比)
     * @param zSpeed percent power for rotation (旋轉的功率百分比)
     * @param fieldOriented configure robot movement style (設置機器運動方式) (field or robot oriented)
     */
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
        SwerveModuleState[] states = null;
        if(fieldOriented) {
            states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                // IMU used for field oriented control
                // IMU 用於 Field Oriented Control
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, mImu.getRotation2d())
            );
        } else {
            states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
            );
        }
        setModuleStates(states);
    }

    /**
     * Get current swerve module states
     * 輸出 4 個 Swerve Module 的當前狀態 modules
     * 
     * @return swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
            mLeftFrontModule.getState(), 
            mRightFrontModule.getState(), 
            mLeftRearModule.getState(), 
            mRightRearModule.getState()
        };
    }

    /**
     * Sets swerve module states
     * 設置 4 個 Swerve module 的狀態。
     * 
     * @param desiredStates array of desired states, order: [leftFront, leftRear, rightFront, rightRear]
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1);
        mLeftFrontModule.setState(desiredStates[0]);
        mRightFrontModule.setState(desiredStates[1]);
        mLeftRearModule.setState(desiredStates[2]);
        mRightRearModule.setState(desiredStates[3]);
    }

    /**
     * Get predicted pose
     * 獲取機器人的當前位置
     * 
     * @return pose
     */
    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    /**
     * Set robot pose
     * 將測程法（odometry）位置設置為給與的 x、y、位置和角度
     * 
     * @param pose robot pose
     */
    public void setPose(Pose2d pose) {
        mOdometry.resetPosition(
            mImu.getRotation2d(),
            new SwerveModulePosition[] {
                mLeftFrontModule.getPosition(),
                mRightFrontModule.getPosition(),
                mLeftRearModule.getPosition(),
                mRightRearModule.getPosition()
            },
            pose);
      }



    
}
