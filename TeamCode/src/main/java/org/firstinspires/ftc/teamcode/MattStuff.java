package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.LightSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Type;

/**
 * Created by matt on 12/10/15.
 */
public class MattStuff {

    private Telemetry Tele;
    private GyroSensor Gyro;
    private DcMotor Left;
    private DcMotor Right;
    private int LeftLastReset = 0;
    private int RightLastReset = 0;
    private ColorSensor Color = null;
    private LightSensor Light = null;


    final int GyroTolerance = 5;

    boolean redAlliance = false;
    boolean delayBeforeStart = false;
    int startPosition = 1;
    boolean parkOnly = false;
    boolean testOnly = false;  // set to true for testing line follower and button pusher

    // constructor
    public MattStuff(Telemetry telemetry, GyroSensor sensorGyro, DcMotor left, DcMotor right, ColorSensor sensorColor) {
        Tele = telemetry;
        Gyro = sensorGyro;
        Left = left;
        Right = right;
        Color = sensorColor;
    }

    // constructor for scrimmagebot
    public MattStuff (Telemetry telemetry, GyroSensor sensorGyro, DcMotor left, DcMotor right, LightSensor sensorLight) {
        Tele = telemetry;
        Gyro = sensorGyro;
        Left = left;
        Right = right;
        Light = sensorLight;
    }

    private int readLineColor() {
        Color.enableLed(true);
        return Color.alpha() + Color.red() + Color.green() + Color.blue();
    }

    private int readLineLight() {
        // Light.enableLed(true);
        return (int) (Light.getLightDetected() * 100);
    }

    boolean onLine() {
        if (Color != null) return (readLineColor() > 20);
        else if (Light != null) return (readLineLight() > 70);
        else return false;
    }

    // heading is never negative, but it can flip from 1 to 359, so do modulus 180
    // If this function returns negative, it means turns right to correct; positive means turn left
    public int SubtractFromCurrHeading(int x) {
        int result = 0;
        int ch = Gyro.getHeading();
        int diff = Math.abs(ch - x);
        if (diff >= 180) { // more than 180deg apart, so flip
            result = 360 - diff;
            if (x < 180) {
                result = -result;
            }
        } else {
            result = ch - x;
        }
        return result;
    }

    public int DiffFromCurrHeading(int x) {
        return Math.abs(SubtractFromCurrHeading(x));
    }

    public int LeftCurrPosition (){
        return Left.getCurrentPosition() - LeftLastReset;
    }

    public int RightCurrPosition (){
        return Right.getCurrentPosition() - RightLastReset;
    }

    public void ResetLeft() {
        LeftLastReset = Left.getCurrentPosition();
    }

    public void ResetRight() {
        RightLastReset = Right.getCurrentPosition();
    }

    public void ResetGyroHeading() throws InterruptedException {

        Gyro.resetZAxisIntegrator();
        while (DiffFromCurrHeading(0) > GyroTolerance*2) {
            Tele.addData("Gyro", String.format("SETTLING, h:%03d", Gyro.getHeading()));
            Thread.sleep(500); // let gyro settle
        }
        Tele.addData("Gyro", String.format("Ready, h:%03d", Gyro.getHeading()));
    }

    public void StopMotors () throws InterruptedException {
        // stop motors and wait for stop
        while (Math.abs(Left.getPower())>0 || Math.abs(Right.getPower())>0){
            Left.setPower(0.0);
            Right.setPower(0.0);
            Thread.sleep(50);
        }
    }

    public void driveStraightByGyroInMeters (double distance, double power) throws InterruptedException {
        double encCount = distance * (6000/ 1.4);
        DriveStraightByGyro((int) encCount, power);
    }

    public void DriveStraightByGyro (int distance, double power) throws InterruptedException {
        double halfPower = 0; //power / 2.0;
        Gyro.resetZAxisIntegrator();
        while (Math.abs(RightCurrPosition()) < distance) {
            if (DiffFromCurrHeading(0) < GyroTolerance/2) {
                // on course, go straight
                Left.setPower(power);
                Right.setPower(power);
            } else { // not on course, correct
                if (SubtractFromCurrHeading(0) < 0) {
                    // correct left
                    Left.setPower(halfPower);
                    Right.setPower(power);
                } else {
                    // correct right
                    Left.setPower(power);
                    Right.setPower(halfPower);
                }
            }
            Tele.addData("Distance: ", distance);
            Tele.addData("Speed", " Left=" + Left.getPower() + " Right=" + Right.getPower());
            Tele.addData("Encoders", " Left=" + LeftCurrPosition() + " Right=" + RightCurrPosition());
            Tele.addData("Straight1", String.format("h:%03d diff:%03d", Gyro.getHeading(), SubtractFromCurrHeading(0)));
        } // while less than encoder target

        // stop motors and wait for stop
        StopMotors();
    }

    public void TurnByGyro (boolean right, int degrees, double power) throws InterruptedException {
        double turnPower = right ? power : - power;
        int turnDegrees = right ? degrees : - degrees;
        Gyro.resetZAxisIntegrator();
        while (DiffFromCurrHeading(turnDegrees) > GyroTolerance) {
            Left.setPower(turnPower);
            Right.setPower(-turnPower);
            Tele.addData("Speed", " Left=" + Left.getPower() + " Right=" + Right.getPower());
            Tele.addData("Encoders", " Left=" + LeftCurrPosition() + " Right=" + RightCurrPosition());
            Tele.addData("Turn1", String.format("h:%03d diff:%03d", Gyro.getHeading(), SubtractFromCurrHeading(turnDegrees)));
        }

        // stop motors and wait for stop
        StopMotors();
    }
    public boolean TurnByGyroAndLineSensor (boolean right, int degrees, double power) throws InterruptedException {
        boolean foundLine = false;
        double turnPower = right ? power : -power;
        int turnDegrees = right ? degrees : -degrees;
        Gyro.resetZAxisIntegrator();
        ResetGyroHeading();
        while (DiffFromCurrHeading(turnDegrees) > GyroTolerance && !foundLine ) {
            Left.setPower(turnPower);
            Right.setPower(-turnPower);
            foundLine = onLine();
            Tele.addData("Speed", " Left=" + Left.getPower() + " Right=" + Right.getPower());
            Tele.addData("Encoders", " Left=" + LeftCurrPosition() + " Right=" + RightCurrPosition());
            Tele.addData("Turn1", String.format("h:%03d diff:%03d", Gyro.getHeading(), SubtractFromCurrHeading(turnDegrees)));
        }
        StopMotors();
        return foundLine;
    }

    public void GetOptionsFromGamepad (Gamepad gamepad) {
        boolean optionsSet = false;
        boolean allianceSet = false;
        boolean delaySet = false;
        boolean startPositionSet = false;
        gamepad.reset();

        while (!optionsSet) {
            Tele.addData("Options", String.format("SELECT, a:%s d:%s p:%d park:%s test:%s" , redAlliance ? "red" : "blue",
                    delayBeforeStart ? "yes": "no", startPosition , parkOnly ? "yes": "no", testOnly ? "yes": "no"));
            if (gamepad.b) {
                redAlliance = true; allianceSet = true;
            } else if (gamepad.x) {
                redAlliance = false; allianceSet = true;
            }
            if (gamepad.y) {
                delayBeforeStart = true; delaySet = true;
            } else if (gamepad.a) {
                delayBeforeStart = false; delaySet = true;
            }
            if (gamepad.dpad_left) {
                startPosition = 1; startPositionSet = true;
            } else if (gamepad.dpad_up) {
                startPosition = 2; startPositionSet = true;
            } else if (gamepad.dpad_down) {
                startPosition = 2; startPositionSet = true;
            } else if (gamepad.dpad_right) {
                startPosition = 3; startPositionSet = true;
            }
            if (gamepad.right_bumper) {
                parkOnly = true;
            }
            if (gamepad.left_bumper) {
                testOnly = true;
            }
            optionsSet = allianceSet && delaySet && startPositionSet;
        }
        Tele.addData("Options", String.format("Set, a:%s d:%s p:%d park:%s test:%s", redAlliance ? "red" : "blue",
                delayBeforeStart ? "yes" :"no", startPosition , parkOnly ? "yes": "no", testOnly ? "yes": "no"));
    }

}
