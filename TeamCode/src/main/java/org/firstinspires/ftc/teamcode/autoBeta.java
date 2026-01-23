/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="autoBeta", group="Robot")
public class autoBeta extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx cuca = null;
    private DcMotorEx juan = null;
    private DcMotorEx pancho = null;
    private DcMotorEx caballo = null;
    private DcMotorEx lapatrona = null;
    private DcMotorEx paquito = null;
    private DcMotorEx kiki = null;

    // Calculate the COUNTS_PER_CM for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_CM   = 8.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * Math.PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        cuca = hardwareMap.get(DcMotorEx.class,"cuca");
        juan = hardwareMap.get(DcMotorEx.class,"juan");
        pancho = hardwareMap.get(DcMotorEx.class,"pancho");
        caballo = hardwareMap.get(DcMotorEx.class,"caballo");


        lapatrona = hardwareMap.get(DcMotorEx.class,"lapatrona");
        paquito = hardwareMap.get(DcMotorEx.class,"paquito");
        kiki = hardwareMap.get(DcMotorEx.class,"kiki");



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        cuca.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        juan.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pancho.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        caballo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kiki.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kiki.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        kiki.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lapatrona.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lapatrona.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lapatrona.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        paquito.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        paquito.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        paquito.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        cuca.setDirection(DcMotorEx.Direction.REVERSE);
        juan.setDirection(DcMotorEx.Direction.FORWARD);
        caballo.setDirection(DcMotorEx.Direction.REVERSE);
        pancho.setDirection(DcMotorEx.Direction.FORWARD);

        lapatrona.setDirection(DcMotorEx.Direction.FORWARD);
        paquito.setDirection(DcMotorEx.Direction.REVERSE);
        kiki.setDirection(DcMotorSimple.Direction.REVERSE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d",
                cuca.getCurrentPosition(),
                juan.getCurrentPosition(),
                pancho.getCurrentPosition(),
                caballo.getCurrentPosition());

        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //Kiki, Estas son lass instrucciones del autonomo, editalas
        kiki.setVelocity(4500);
        waitS(3);
        paquito.setVelocity(2500);
        waitS(1.5);
        lapatrona.setVelocity(2500);
        waitS(5);
        kiki.setVelocity(0);
        lapatrona.setVelocity(0);
        paquito.setVelocity(0);
        encoderDrive(0.5, -100, 0, 0, 5);
        encoderDrive(0.5, 0, 0, -25, 5);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double driveCm, double strafeCm, double turnCm,
                             double timeoutS) {
        int newCaballoTarget;
        int newPanchoTarget;
        int newCucaTarget;
        int newJuanTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newCaballoTarget = caballo.getCurrentPosition() + (int)((driveCm-strafeCm+turnCm) * COUNTS_PER_CM);
            newPanchoTarget = pancho.getCurrentPosition() + (int)((driveCm+strafeCm-turnCm) * COUNTS_PER_CM);
            newCucaTarget = cuca.getCurrentPosition() + (int)((driveCm+strafeCm+turnCm) * COUNTS_PER_CM);
            newJuanTarget = juan.getCurrentPosition() + (int)((driveCm-strafeCm-turnCm) * COUNTS_PER_CM);

            caballo.setTargetPosition(newCaballoTarget);
            pancho.setTargetPosition(newPanchoTarget);
            cuca.setTargetPosition(newCucaTarget);
            juan.setTargetPosition(newCaballoTarget);


            // Turn On RUN_TO_POSITION
            caballo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            pancho.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            cuca.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            juan.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            caballo.setPower(Math.abs(speed));
            pancho.setPower(Math.abs(speed));
            cuca.setPower(Math.abs(speed));
            juan.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (caballo.isBusy() && pancho.isBusy() && cuca.isBusy() && juan.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newCaballoTarget,  newPanchoTarget, newCucaTarget,  newJuanTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d ",
                        caballo.getCurrentPosition(), pancho.getCurrentPosition(), cuca.getCurrentPosition(), juan.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            caballo.setPower(0);
            pancho.setPower(0);
            cuca.setPower(0);
            juan.setPower(0);

            // Turn off RUN_TO_POSITION
            caballo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            pancho.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            cuca.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            juan.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void waitS(double timeS){
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeS) {
            // Just wait for 1 second
        }
    }
}
