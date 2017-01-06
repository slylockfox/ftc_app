
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

@TeleOp(name="Caroline Chase IR", group="Caroline")
public class CarolineChaseIR extends OpMode{
    private DcMotor motorLeft, motorRight;
    private DcMotor motorLinearScissor, motorScrewScissor;
    private TouchSensor touchBottomStop, touchTopStop;
    private OpticalDistanceSensor distance_sensor;
    private UltrasonicSensor sonar;
    private IrSeekerSensor ir;
    private LightSensor lightSensor;
    private GyroSensor sensorGyro;
    private MattStuff mattStuff;

    private final double DEADZONE = 0.1;
    private final double IR_ANGLE_TOLERANCE = 10.0;  // degrees
    private final double IR_THRESHOLD = 0.1;
    private final double SONAR_THRESHOLD = 100; // less means obstascle is in front of us

    @Override
    public void init() {
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");
        motorLinearScissor = hardwareMap.dcMotor.get("lup");
        motorScrewScissor = hardwareMap.dcMotor.get("rup");
        distance_sensor = hardwareMap.opticalDistanceSensor.get("distance");
        touchBottomStop = hardwareMap.touchSensor.get("touch1");
        touchTopStop = hardwareMap.touchSensor.get("touch0");
        sonar = hardwareMap.ultrasonicSensor.get("sonar");
        ir = hardwareMap.irSeekerSensor.get("ir");
        lightSensor = hardwareMap.lightSensor.get("light");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");


        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distance_sensor.enableLed(true);

        mattStuff = new MattStuff(telemetry, sensorGyro, motorLeft, motorRight, lightSensor);
        try {
            mattStuff.ResetGyroHeading();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop(){

        double l, r;
        double b, lV, lH;

        double sbucket = 160.0/255.0;
        boolean lastTurnedRight = false;

        // extend IR stalk if dpad button pushed, or nobody is in front of robot
        // retract IR stalk if dbad button pushed, or robot is close to someone/something
        boolean retractIR = false;
        boolean extendIR = false;
        if (gamepad1.dpad_down) retractIR = true;
        else if (gamepad1.dpad_up) extendIR = true;
        else if (sonar.getUltrasonicLevel() < SONAR_THRESHOLD) retractIR = true;
        else if (sonar.getUltrasonicLevel() >= SONAR_THRESHOLD) extendIR = true;

        if(Math.abs(gamepad1.left_stick_y) > DEADZONE){
            l = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);  // square it and trim it
            retractIR = true; extendIR = false;  // bring IR in when drivers are driving
        }else{ l = 0;}
        if(Math.abs(gamepad1.right_stick_y) > DEADZONE){
            r = -gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y); // square it and trim it
            retractIR = true; extendIR = false;  // bring IR in when drivers are driving
        }else{ r = 0;}

        // if no controls from driver, then follow IR beacon, if it's on
        if (l == 0 && r == 0) {
            if (!touchBottomStop.isPressed() && ir.getStrength() > IR_THRESHOLD) {
                // extended and IR signal detected
                if (Math.abs(ir.getAngle()) < IR_ANGLE_TOLERANCE) { // go straight
                    l = 1.0;
                    r = 1.0;
                } else { // turn toward beacon
                    if (ir.getAngle() > 0.0) { // turn right
                        lastTurnedRight = true;
                        l = 1.0;
                        r = 0.1;
                    } else { // turn left
                        lastTurnedRight = false;
                        l = 0.1;
                        r = 1.0;
                    }
                } // turning toward beacon
            } else if (touchTopStop.isPressed()) { // no driver, no IR, sensor retracted
                if (sonar.getUltrasonicLevel() < SONAR_THRESHOLD && mattStuff.DiffFromCurrHeading(0) > 5) {
                    // obstace in front of us, swivel until clear
                    if (lastTurnedRight) { // turned right, into person maybe, so turn back left
                        l = -1.0;
                        r = 1.0;
                    } else { // turned left, into person maybe, so turn back right
                        l = 1.0;
                        r = -1.0;
                    }
                } else { // nothing in front of us, or we swiveled back to start, so rest
                    l = 0.0;
                    r = 0.0;
                }
            } // not extended
        } // if no driver control

        if(gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1){
            lV = gamepad2.left_stick_y;
        }else{ lV = 0;}
        if(gamepad2.right_stick_x > 0.1 || gamepad2.right_stick_x < -0.1){
            lH = gamepad2.right_stick_x;
        }else{ lH = 0;}

        if(extendIR && touchBottomStop.isPressed()){ b = .3;}
        else if(retractIR && !touchTopStop.isPressed()){ b = -.1;}
        else{ b = 0.0;}

        if(gamepad2.a){
            sbucket = 250.0/255.0; // for a continuous servo, above 128/255 means forward
        }else if(gamepad2.b){
            sbucket = 128.0/255.0; // for a continous servo, 128/255 = 0.5 = stop
        } else {
            sbucket = 100.0/255.0; // for a continuous servo, below 128/255 means reverse
        }

        motorLeft.setPower(l);
        motorRight.setPower(r);
        motorScrewScissor.setPower(lV);
        motorLinearScissor.setPower(b);

        telemetry.addData("Left", String.format("Power:%.1f, Enc:%03d", l, mattStuff.LeftCurrPosition()));
        telemetry.addData("Right", String.format("Power:%.1f, Enc:%03d", r, mattStuff.RightCurrPosition()));
        //telemetry.addData("Screw", lV);
        //telemetry.addData("Linear", b);
        telemetry.addData("Sonar", sonar.getUltrasonicLevel());
        telemetry.addData("IR Angle", ir.getAngle());
        telemetry.addData("IR Strength", ir.getStrength());
    }
}