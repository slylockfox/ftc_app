/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

/*
 * This is an example LinearOpMode that shows how to use
 * the Modern Robotics ITR Seeker
 *
 * The op mode assumes that the IR Seeker
 * is configured with a name of "sensor_ir".
 *
 * Set the switch on the Modern Robotics IR beacon to 1200 at 180.  <br>
 * Turn on the IR beacon.
 * Make sure the side of the beacon with the LED on is facing the robot. <br>
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Test IR Seekers2", group = "Uselessbot")
//@Disabled
public class UselessTestMRIrSeeker extends LinearOpMode {

    // "complete" solution from Nick Spence for #360, multiple same I2C sensors
    private void setI2cAddress(ModernRoboticsI2cIrSeekerSensorV3 sensor, int addr)
    {
        sensor.setI2cAddress(I2cAddr.create8bit(addr));
    }

    @Override
    public void runOpMode() {

        IrSeekerSensor  irSeekerH;    // Hardware Device Object
        ModernRoboticsI2cIrSeekerSensorV3  irSeekerV;
        I2cDevice       irSeekerVDevice;
        I2cDeviceSynch  irSeekerVreader;

        // get a reference to our IR Seeker object.
        //irSeekerV = hardwareMap.irSeekerSensor.get("seekerV");
//        irSeekerV.setI2cAddress(I2cAddr.create7bit(0x1E));
        // irSeekerV.setI2cAddress(I2cAddr.create8bit(0x42));
        //irSeekerV.setI2cAddress(I2cAddr.create8bit(0x42));

        irSeekerH = hardwareMap.irSeekerSensor.get("seekerH");

        // "complete" solution from Nick Spence for #360, multiple same I2C sensors
        irSeekerVDevice = hardwareMap.i2cDevice.get("seekerV");
        irSeekerVreader = new I2cDeviceSynchImpl(irSeekerVDevice, I2cAddr.create8bit(0x0), false);  // was I2cDeviceSynchImpl2
        irSeekerV = new ModernRoboticsI2cIrSeekerSensorV3(irSeekerVreader);


        // wait for the start button to be pressed.
        waitForStart();

        setI2cAddress(irSeekerV, 0x42);  // "complete" solution from Nick Spence for #360, multiple same I2C sensors

        while (opModeIsActive())  {

            // Ensure we have a IR signal
            if (irSeekerH.signalDetected())
            {
                // Display angle and strength
                telemetry.addData("AngleH",    irSeekerH.getAngle());
                telemetry.addData("StrengthH", irSeekerH.getStrength());
            }
            else
            {
                // Display loss of signal
                telemetry.addData("Seeker H", "Signal Lost");
            }
            if (irSeekerV.signalDetected())
            {
                // Display angle and strength
                telemetry.addData("AngleV",    irSeekerV.getAngle());
                telemetry.addData("StrengthV", irSeekerV.getStrength());
            }
            else
            {
                // Display loss of signal
                telemetry.addData("Seeker V", "Signal Lost");
            }

            telemetry.update();
        }
    }
}
