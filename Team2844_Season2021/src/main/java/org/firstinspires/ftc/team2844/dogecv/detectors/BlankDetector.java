package org.firstinspires.ftc.team2844.dogecv.detectors;

import org.opencv.core.Mat;

/**
 * Created by Victo on 12/17/2017.
 */

public class BlankDetector extends DogeCVDetector {
    @Override
    public Mat process(Mat input) {
        // Process frame
        return input;
    }

    @Override
    public void useDefaults() {
        // Add in your scorers here.
    }
}
