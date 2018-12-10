package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareUselessbot
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public CRServo swivelServo = null;
    public Servo shoulderServo = null;
    public Servo    buttonPusher    = null;
    public AnalogInput potentiometer = null;
    public ColorSensor colorSensor = null;
    public ModernRoboticsI2cRangeSensor rangeSensor = null;
    private IrSeekerSensor irSeekerH;
    private IrSeekerSensor irSeekerV;
    public boolean swapSeekers = false;
    private I2cDeviceSynch irSeekerVreader;
    public static final double PUSHER_RETRACT   =  0.5 ;
    public static final double PUSHER_EXTEND   =  0.2 ;
    public static final double ARM_STOP_POWER    =  0.0 ;
    public static final double ARM_RIGHT_POWER  = 1.0 ;
    public static final double ARM_LEFT_POWER  = -1.0 ;
    public static final double SHOULDER_LEVEL = .15;
    public static final double ARM_FRONT = 34.3;
    public static final double ARM_RIGHT = 38.1;
    public static final double ARM_REAR = 27.7;
    public static final double ARM_LEFT = 30.4;
    public static final double ARM_TOLERANCE = 0.2;
    public static final double IR_ANGLE_TOLERANCE_H = 65; // 70?
    public static final double IR_ANGLE_TOLERANCE_V = 50; // 60?


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private boolean ReverseDriveMotors = false;

    /* Constructor */
    public HardwareUselessbot(){
    }

    public HardwareUselessbot(boolean reverse){
        ReverseDriveMotors = reverse;
    }

    public IrSeekerSensor seekerH() {if (swapSeekers) return irSeekerV; else return irSeekerH;}
    public IrSeekerSensor seekerV() {if (swapSeekers) return irSeekerH; else return irSeekerV;}
    public void toggleSeekerSwap() {swapSeekers = !swapSeekers;}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        // MJS was... armMotor    = hwMap.dcMotor.get("left_arm");

        if (ReverseDriveMotors) {
            leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
            rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        } else {
            leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        }

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        swivelServo = hwMap.crservo.get("swivel");
        buttonPusher = hwMap.servo.get("pusher");
        buttonPusher.setPosition(PUSHER_RETRACT);
        shoulderServo = hwMap.servo.get("shoulder");
        shoulderServo.setPosition(SHOULDER_LEVEL);

        // Sensors
        //rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        potentiometer = hwMap.analogInput.get("potentiometer");
        //colorSensor = hwMap.colorSensor.get("color");
        //colorSensor.enableLed(false);

        //solution per Nick Spence for issue #360 and two IR sensors with different addresses
        // note synchronous device seems to be depracated, but there's no other way to get two IR seekers
//        irSeekerVreader = hwMap.i2cDeviceSynch.get("seekerV");
//        irSeekerV = new ModernRoboticsI2cIrSeekerSensorV3(irSeekerVreader);

        irSeekerH = hwMap.irSeekerSensor.get("seekerH");
        irSeekerV = hwMap.irSeekerSensor.get("seekerV");
        irSeekerH.setI2cAddress(I2cAddr.create8bit(0x42));

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public double armPosition() {
        double voltreading = (float) potentiometer.getVoltage();
        double percentTurned = voltreading/5 * 100;
        return percentTurned;
    }

    public double IRMovementH() {
        double currentPos = armPosition();
        double angle = seekerH().getAngle();
        double result = ARM_STOP_POWER;
        if (Math.abs(angle) > IR_ANGLE_TOLERANCE_H) {
            if (angle > 0 && currentPos < ARM_RIGHT)
                result = ARM_RIGHT_POWER;
            else if (angle < 0 && currentPos > ARM_REAR)
                result = ARM_LEFT_POWER;
            else
                result = ARM_STOP_POWER;
        }
        return result;
    }

    public double IRMovementV() { // positive, negative, or zero
        double angle = seekerV().getAngle();
        double result = 0.0;
        if (Math.abs(angle) > IR_ANGLE_TOLERANCE_V) {
            //result = angle < 0 ? 1 : -1;
            result = angle / (IR_ANGLE_TOLERANCE_V * 2);
        }
        return result;
    }

    public double powerToPosition(double target) {
        double currentPos = armPosition();
        double result = ARM_STOP_POWER;
        if (Math.abs(currentPos - target) > ARM_TOLERANCE) {
            if (currentPos > ARM_RIGHT) { // don't swing any further right; go back
                result = ARM_LEFT_POWER;
            } else if (currentPos < ARM_REAR) { // don't swing any further left; go back
                result = ARM_RIGHT_POWER;
            } else { // ok to swing either way
                if (target > currentPos)
                    result = ARM_RIGHT_POWER;
                else
                    result = ARM_LEFT_POWER;
            }
        }
        return result;
    }
}

