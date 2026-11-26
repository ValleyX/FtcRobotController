package org.firstinspires.ftc.team2844.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vcs.valleylib.ftc.hardware.FtcSubsystem;

import org.firstinspires.ftc.team2844.helpers.Constants;
import org.firstinspires.ftc.team2844global.Ball;
import org.firstinspires.ftc.team2844global.GlobalConstants;
import org.firstinspires.ftc.team2844global.MatchInfo;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

public class HuskySubsystem extends FtcSubsystem {
    private final int TOP_CUTTOFF = 70;
    private final int CAMERA_WIDTH = 320;
    private final int CAMERA_HALF_WIDTH = CAMERA_WIDTH/2;
    private final int CAMERA_HEIGHT = 240;
    private final int CAMERA_HALF_HEIGHT = CAMERA_HEIGHT/2;
    private final double CAMERA_TILT = 15;
    private final double HEIGHT_ON_ROBOT = 12.0;

    private final double vFOV = 40.0;
    private final double hFOV = 50.0;
    private final double FORWARD_OFFSET = 6.0; //TODO put the amount forward on the bot the camera is
    private final double RIGHT_OFFSET = 0.0; //TODO put the amount right the camera is on the bot
    /**How far a ball can move between two camera frames and still count as the same ball, in inches.
     * Too small loses fast balls, too big mixes up balls that are close together.*/
    private final double MAX_MATCH_DISTANCE = 5.0; //TODO tune

    private final HuskyLens huskyLens;
    public HuskySubsystem(HardwareMap hardwareMap){
        super(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, Constants.CHI2C2);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public List<Pose> getEndPoses(Pose currentPose){
        List<Pose> endPoses = new ArrayList<>();
        //get ball poses
        //track which ones are which and set their new poses
        //get the predicted end poses
        List<Ball> balls = getVisibleBalls(MatchInfo.alliance, currentPose);
        List<Ball> prevBalls = List.copyOf(balls);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (timer.time(TimeUnit.MILLISECONDS) < 500){
            //get the current balls
            List<Ball> loopBalls = getVisibleBalls(MatchInfo.alliance, currentPose);

            //find the closest for each, then check if they are valid, if they are, set the track ids to the same as the closestBall
            setTrackIDs(loopBalls, prevBalls);

            Set<Integer> currentIDs = loopBalls.stream()
                    .map(Ball::getTrackID)
                    .collect(Collectors.toSet());

            List<Ball> missingBalls = prevBalls.stream()
                    .filter(b -> !currentIDs.contains(b.getTrackID()))
                    .collect(Collectors.toList());

            prevBalls = new ArrayList<>(loopBalls);
            prevBalls.addAll(missingBalls);
        }

        filterForTracks(prevBalls);
        //compare the track ids and set the new poses
        setPoseWithTrack(balls, prevBalls);
        //predict all the end poses and return
        endPoses = getPredictedList(balls);

        return endPoses;
    }

    /**One possible match between an old ball and a new ball, used by setTrackIDs.*/
    private static class TrackMatch {
        final int oldIndex;
        final int newIndex;
        final double distance;

        TrackMatch(int oldIndex, int newIndex, double distance){
            this.oldIndex = oldIndex;
            this.newIndex = newIndex;
            this.distance = distance;
        }
    }

    /**Gives each new ball (modified) the track ID of the old ball (modifier) it matches.
     * Only balls of the same color closer than MAX_MATCH_DISTANCE can match. The closest pairs
     * go first, and each old ball and each new ball can only be used once
     * New balls with no match keep their own ID.*/
    public void setTrackIDs(List<Ball> modified, List<Ball> modifier){
        //every possible match
        List<TrackMatch> matches = new ArrayList<>();
        for (int k = 0; k < modifier.size(); k++) {
            for (int j = 0; j < modified.size(); j++) {
                double distance = modifier.get(k).distance(modified.get(j));
                if (distance < MAX_MATCH_DISTANCE && modifier.get(k).getColorID() == modified.get(j).getColorID()) {
                    matches.add(new TrackMatch(k, j, distance));
                }
            }
        }

        //closest first
        matches.sort(Comparator.comparingDouble(m -> m.distance));

        boolean[] oldUsed = new boolean[modifier.size()];
        boolean[] newUsed = new boolean[modified.size()];
        for (TrackMatch m : matches) {
            if (!oldUsed[m.oldIndex] && !newUsed[m.newIndex]) {
                modified.get(m.newIndex).setTrackID(modifier.get(m.oldIndex).getTrackID());
                oldUsed[m.oldIndex] = true;
                newUsed[m.newIndex] = true;
            }
        }
    }
    public void filterForTracks(List<Ball> balls){
        Set<Integer> seenIds = new HashSet<>();

        // Removes any object whose 'id' has already been added to the set
        balls.removeIf(obj -> !seenIds.add(obj.getTrackID()));
    }
    public void setPoseWithTrack(List<Ball> modified, List<Ball> modifier){
        for (int i = 0; i < modified.size(); i++){
            for (int h = 0; h < modifier.size(); h++){
                if(modified.get(i).getTrackID() == modifier.get(h).getTrackID()){
                    modified.get(i).setPose(modifier.get(h));
                }
            }
        }
    }

    public List<Pose> getPredictedList(List<Ball> balls){
        List<Pose> poses = new ArrayList<>();
        for (int i = 0; i < balls.size(); i++){
            Pose endPose = balls.get(i).predictEndPose();
            //null means it bounces off a corner and can't be predicted. A later snapshot will see it once it stops.
            if(endPose != null){
                poses.add(endPose);
            }
        }
        return poses;
    }

    public List<Ball> getVisibleBalls(int alliance, Pose currentPose){
        int nectarID = (alliance == GlobalConstants.BLUE_ALLIANCE? GlobalConstants.BLUE_NECTAR: GlobalConstants.RED_NECTAR);

        ArrayList<HuskyLens.Block> blobs = new ArrayList<>(List.of(huskyLens.blocks()));

        preFilter(blobs, nectarID);

        List<Ball> balls = new ArrayList<>();
        for(int j = 0; j < blobs.size(); j++){
            double blobForward = getForwardInches(blobs.get(j)) ; //inches forward
            double blobRight = getHorizontalInches(blobs.get(j), blobForward); //inches to the side (Left is negative)

            blobForward = FORWARD_OFFSET + blobForward;
            blobRight = RIGHT_OFFSET + blobRight;

            double x = currentPose.x() + (blobForward*Math.cos(currentPose.heading())) + (blobRight*Math.sin(currentPose.heading()));
            double y = currentPose.y() + (blobForward*Math.sin(currentPose.heading())) - (blobRight*Math.cos(currentPose.heading()));
            balls.add(new Ball(new Pose(x,y), blobs.get(j).id));
        }

        postFilter(balls);

        return sortClosest(currentPose, balls);
    }

    private void preFilter(ArrayList<HuskyLens.Block> blobs, int nectarID){
        for(int i = blobs.size() - 1; i >= 0; i--){
            if(blobs.get(i).y < TOP_CUTTOFF
                    || (blobs.get(i).id != GlobalConstants.POLLEN
                    && blobs.get(i).id != nectarID)){
                blobs.remove(i);
            }
        }
    }

    private void postFilter(List<Ball> balls){
        for(int i = balls.size() - 1; i >= 0; i--){
            Ball b = balls.get(i);
            //?remove it if it is outside the field
            if((b.getPose().x() > GlobalConstants.FIELD_SIZE || b.getPose().y() > GlobalConstants.FIELD_SIZE) || (b.getPose().x() < 0.0 || b.getPose().y() < 0.0)){
                balls.remove(i);
            }
        }
    }

    public double getForwardInches(HuskyLens.Block blob){
        double radius = (blob.id == GlobalConstants.POLLEN)? 1.4: 1.8;
        double fromCenter = (blob.y - CAMERA_HALF_HEIGHT)/(double)CAMERA_HALF_HEIGHT;
        double downAngle = Math.toRadians(CAMERA_TILT) + Math.atan(fromCenter * Math.tan(Math.toRadians(vFOV)/2.0));
        double slope = Math.tan(downAngle);
        return (HEIGHT_ON_ROBOT - radius)/slope;
    }

    public double getHorizontalInches(HuskyLens.Block blob){
        return getHorizontalInches(blob, getForwardInches(blob));
    }
    public double getHorizontalInches(HuskyLens.Block blob, double forwardInches){
        double fromCenter = (blob.x - CAMERA_HALF_WIDTH)/(double) CAMERA_HALF_WIDTH;
        double radius = (blob.id == GlobalConstants.POLLEN)? 1.4: 1.8;
        double depthWithTilt = forwardInches*Math.cos(Math.toRadians(CAMERA_TILT)) + (HEIGHT_ON_ROBOT - radius)*Math.sin(Math.toRadians(CAMERA_TILT));
        double slope = fromCenter*Math.tan(Math.toRadians(hFOV)/2.0);
        return slope*depthWithTilt;
    }

    public List<Ball> sortClosest(Pose currentPose, List<Ball> balls){
//        List<Pose> sorted = new ArrayList<>();
//        for (int i = poses.size()-1; i >= 0; i--){
//            double minLength = currentPose.distance(poses.get(i));
//            int minIndex = i;
//            for(int j = 0; j < poses.size()-1; j++){
//                double distance = currentPose.distance(poses.get(j));
//                if(distance < minLength){
//                    minLength = distance;
//                    minIndex = j;
//                }
//            }
//            sorted.add(poses.get(minIndex));
//            poses.remove(minIndex);
//        }
//        return sorted;
        balls.sort(Comparator.comparingDouble(b->currentPose.distance(b.getPose())));
        return balls;
    }
}
