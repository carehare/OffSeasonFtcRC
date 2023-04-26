/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.framework.Utilities;

import org.opencv.core.Point;

import java.util.ArrayList;

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple stones, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@TeleOp
public class FindConeOpMode extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        Utilities.getSharedUtility().initialize(this);

        RobotVision robotVision = new RobotVision(1);

        // Tell telemetry to update faster than the default 250ms period :)
//        telemetry.setMsTransmissionInterval(20);

        waitForStart();

        while (opModeIsActive()) {
            Log.v("pipeline", "loop started");
            // Don"t burn an insane amount of CPU cycles in this sample because
            // we"re not doing anything else
            sleep(20);
            telemetry.addLine(String.format("Color in teal: %d", robotVision.getSampleColor()));
            Point conePosition = robotVision.getConePosition();
            telemetry.addLine(String.format("Cone position: %f, %f", conePosition.x, conePosition.y));
            telemetry.update();
            Log.v("pipeline", "loop will end");
        }
    }
}

