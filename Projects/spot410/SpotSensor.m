classdef SpotSensor < Simulink.IntEnumType
    enumeration
        xRedPhasespace        (SpotCoord.xRed)
        yRedPhasespace        (SpotCoord.yRed)
        thetaRedPhasespace    (SpotCoord.thetaRed)
        xBlackPhasespace      (SpotCoord.xBlack)
        yBlackPhasespace      (SpotCoord.yBlack)
        thetaBlackPhasespace  (SpotCoord.thetaBlack)
        xBluePhasespace       (SpotCoord.xBlue)
        yBluePhasespace       (SpotCoord.yBlue)
        thetaBluePhasespace   (SpotCoord.thetaBlue)
        shoulderArmPhasespace (SpotCoord.shoulderArm)
        elbowArmPhasespace    (SpotCoord.elbowArm)
        wristArmPhasespace    (SpotCoord.wristArm)
%
%       % offset of 12 corresponds to the number of PhaseSpace measurements
        xRedImu       (12 + SpotCoord.xRed)
        yRedImu       (12 + SpotCoord.yRed)
        thetaRedImu   (12 + SpotCoord.thetaRed)
        xBlackImu     (12 + SpotCoord.xBlack)
        yBlackImu     (12 + SpotCoord.yBlack)
        thetaBlackImu (12 + SpotCoord.thetaBlack)
        xBlueImu      (12 + SpotCoord.xBlue)
        yBlueImu      (12 + SpotCoord.yBlue)
        thetaBlueImu  (12 + SpotCoord.thetaBlue)
%
%       % offset of 9 corresponds to the number of IMU measurements
        xStereo     (12 + 9 + 1)
        yStereo     (12 + 9 + 2)
        thetaStereo (12 + 9 + 3)
        xLidar      (12 + 9 + 4)
        yLidar      (12 + 9 + 5)
        thetaLidar  (12 + 9 + 6)
        rLaser      (12 + 9 + 7)
    end
end

