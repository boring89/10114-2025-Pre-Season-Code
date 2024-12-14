package frc.robot;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class Configs {
    
    public static final class SwerveModule {
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

        static {
            
        }
    }
}
