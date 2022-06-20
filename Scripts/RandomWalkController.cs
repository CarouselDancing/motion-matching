using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Carousel
{

namespace MotionMatching{
    
public enum ControllerMode
{
    OLD,
    SYNC,
    INPUT
}
public class RandomWalkController : MMPoseProvider
{

    PoseState oldPose;
    PoseState refPose;
    public MMControllerSettigs settings;
    public CameraController cameraController;
    public bool prediction;
    public int frameIdx = 0;
    public float FPS = 60;
    public bool active = true;
    public float syncTimer = 0;
    float forceSearchTimer = 0.1f;
    public int nnDistributionSize = 100;
    public float interval;
    public float dt;
    public ControllerMode mode;
    public bool drawQuery;
    public bool activateSimBone = true;
    public bool drawGizmo = true;
    public float timeScale = 1;
    void Start()
    {
        Time.timeScale = timeScale;
        poseState = mm.Load();
        refPose = new PoseState(mm.database.nBones, mm.database.boneParents);
        oldPose = new PoseState(mm.database.nBones, mm.database.boneParents);
        mm.ComputeFeatures();

        interval = 1.0f / FPS;
        syncTimer = interval;
        prediction = false;
        frameIdx = settings.startFrameIdx;

    }

    void Update()
    {
        dt = Time.deltaTime* Time.timeScale;
        if (!active) return;
        interval = 1.0f / FPS;
        switch (mode)
        {
            case ControllerMode.SYNC:
                if (syncTimer > 0)
                {
                    syncTimer -= dt;
                    return;
                }
                syncTimer = interval;
                Step();
                break;
            case ControllerMode.OLD:
                Step();
                break;
            case ControllerMode.INPUT:
                if(Input.GetKeyDown(KeyCode.Space)) Step();
                break;

        }
        SetPose();
        if (activateSimBone) { 
            transform.position = poseState.simulationPosition;
            transform.rotation = poseState.simulationRotation;
        }
    }

    public void Step()
    {
        if (forceSearchTimer > 0)
        {
            forceSearchTimer -= interval;
        }
        bool end_of_anim = false;
        try
        {
            end_of_anim = mm.trajectoryIndexClamp(frameIdx, 1) == frameIdx;
        }
        catch (Exception e)
        {

        }
        //poseState.simulationPosition += poseState.simulationRotation*poseState.simulationVelocity*interval;
        //poseState.simulationRotation = Quat.from_scaled_angle_axis(poseState.simulationAV * interval) * poseState.simulationRotation;


        Vector3 desiredVelocity = mm.database.boneVelocities[frameIdx, 0];
        // desiredVelocity.x *= -1;

        Quaternion desiredRotation = mm.database.boneRotations[frameIdx, 0];
        //desiredRotation.x *= -1;
        //desiredRotation.w *= -1;
        simulationPositionsUpdate(ref poseState.simulationPosition, ref poseState.simulationVelocity, ref poseState.simulationAcceleration, desiredVelocity, settings.simulationVelocityHalflife, interval);
        simulationRotationsUpdate(ref poseState.simulationRotation, ref poseState.simulationAV, desiredRotation, settings.simulationRotationHalflife, interval);

        if (end_of_anim || forceSearchTimer <= 0.0f)
        {
            //FindTransition();
            forceSearchTimer = settings.forceSearchTime;
            prediction = true;
        }
        else
        {
            prediction = false;
        }
        frameIdx++;//prevents getting stuck
        if (frameIdx >= mm.database.nFrames)
        {
            frameIdx = 0;
        }
    }


    public void FindTransition()
    {
        int oldFrameIdx = frameIdx;
        //frameIdx = mm.FindTransition(poseState, frameIdx);
        var result = new SearchResult(nnDistributionSize);
        mm.FindTransition(poseState, frameIdx, ref result);

        //frameIdx = result.sortedList.Values[0];
        var bestIdx = result.GetBestIndex();
        frameIdx = result.GetRandomIndex(oldFrameIdx);
        //Debug.Log("new frame idx"+ frameIdx.ToString() + " " + bestIdx.ToString());
        if (drawQuery)
        {
            oldPose.SetState(mm.database, oldFrameIdx, false);
            refPose.SetState(mm.database, frameIdx, false);
        }
    }


    public void SetPose()
    {
       
        poseState.SetState(mm.database, frameIdx, true, false);
    }


    public void OnDrawGizmos()
    {
        if (!drawGizmo) return;
        Gizmos.color = Color.green;
        if (prediction)
        {
            Gizmos.color = Color.red;
        }
        if (poseState != null) poseState.Draw(settings.visScale);
        if (drawQuery) { 
            Gizmos.color = Color.yellow;
            if (refPose != null) refPose.Draw(settings.visScale);
            Gizmos.color = Color.red;
            if (oldPose != null) oldPose.Draw(settings.visScale);
        }
    }
  
  override public void ResetToIdle(){

  }

}
}
}
