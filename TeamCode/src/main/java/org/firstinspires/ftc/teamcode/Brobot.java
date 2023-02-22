package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class Brobot extends SampleMecanumDrive {
    //hardware map, rr drive, timer and logs
    private final HardwareMap hardwareMap;

    //region Hardware Classes Initialization
    public Motor leftFront, leftBack, rightFront, rightBack;

    public OpenCvPipeline pipeline;
    public static OpenCvInternalCamera webCam;
    public static final int CAMERA_WIDTH = 320;
    public static final int CAMERA_HEIGHT = 240;

    /**
     * Initialize a new Brobot class. Sets up odometry and hardware components.
     * @param map Hardware Map to access hardware components.
     */
    public Brobot(HardwareMap map){
        //Create Mecanum Drive Class.
        super(map);

        hardwareMap = map;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        /*
        //Initialize Encoders for Localizer.
        leftEncoder = new MotorEx(map, "rightFront");
        rightEncoder = new MotorEx(map, "leftFront");
        backEncoder = new MotorEx(map, "leftBack");
        leftEncoder.setDistancePerPulse(-DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        backEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        imu = new RevIMU(map);
        imu.init();
        localizer = new HolonomicIMUOdometry(leftEncoder::getDistance, rightEncoder::getDistance, backEncoder::getDistance,
                trackWidth, centerWheelOffset, imu);
         */
        //localizer = new T265Localizer(new Pose2d(40,40, new Rotation2d(Math.toDegrees(90))), hardwareMap, "rightFront", "leftFront", "leftBack");

        //region Hardware Initialization
        leftFront = new Motor(hardwareMap, "leftFront");
        leftBack = new Motor(hardwareMap, "leftBack");
        rightFront = new Motor(hardwareMap, "rightFront");
        rightBack = new Motor(hardwareMap, "rightBack");
    }
}
