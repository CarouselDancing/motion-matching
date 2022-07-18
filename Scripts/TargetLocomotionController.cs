using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Carousel.MotionMatching{
    
public class TargetLocomotionController : LocomotionController
{
    public Transform target;
    public bool orientTowardsTarget;

    void Start()
    {
        
        var anim = GetComponent<Animator>();
        if(anim != null) anim.enabled = false;
        poseState = mm.Load();
        poseState.simulationPosition = transform.position;
         poseState.simulationPosition.y = 0;
        poseState.simulationRotation = transform.rotation;
        refPose = new PoseState(mm.database.nBones, mm.database.boneParents);
        mm.ComputeFeatures();

        float interval = 1.0f / FPS;
        syncTimer = interval;
        prediction = false;
        frameIdx = settings.startFrameIdx;

    }



    override public void Step(float interval)
    {

        if (forceSearchTimer > 0)
        {
            forceSearchTimer -= interval;
        }


        desiredVelocity = UpdateDesiredVelocity(0);

        desiredRotation = UpdateDesiredRotation(desiredVelocity, 0);

        float predictionDt = interval * settings.predictionDistance;

        UpdateDesiredRotationTrajectory(predictionDt);
        UpdatePredictedRotationTrajectory(predictionDt);

        UpdateDesiredVelocityTrajectory(predictionDt);
        UpdatePredictedPositionTrajectory(predictionDt);


        //update simulation
        float dt = interval;
        Vector3 velocity = desiredVelocity;
        Quaternion rotation = desiredRotation;
        if(useMotionVelocity){
            velocity = poseState.simulationRotation* poseState.motionVelocity*Time.deltaTime;
            var av = poseState.simulationRotation* poseState.motionAV*Time.deltaTime;
            poseState.simulationPosition += velocity*velocityScale;
            poseState.simulationRotation *= Quaternion.Euler(av.x*Mathf.Rad2Deg, av.y*Mathf.Rad2Deg, av.z*Mathf.Rad2Deg);
        }else{
            simulationPositionsUpdate(ref poseState.simulationPosition, ref poseState.simulationVelocity, ref poseState.simulationAcceleration, velocity, settings.simulationVelocityHalflife, dt);
            simulationRotationsUpdate(ref poseState.simulationRotation, ref poseState.simulationAV, rotation, settings.simulationRotationHalflife, dt);
        }
        
        bool endOfAnim = mm.trajectoryIndexClamp(frameIdx, 1) == frameIdx;
        
        if (endOfAnim || forceSearchTimer <= 0.0f)
        {
            FindTransition();
            forceSearchTimer = settings.forceSearchTime;
            prediction = true;
        }
        else
        {
            prediction = false;
        }
        SetPose();
    
        refPose.SetState(mm.database, frameIdx, false);

        frameIdx++;//prevents getting stuck
        verifyFrame();

        transform.position = poseState.simulationPosition;
        transform.rotation = poseState.simulationRotation;

    }



    override public void FindTransition()
    {
        int oldFrameIdx = frameIdx;
        frameIdx = mm.FindTransition(poseState, frameIdx, trajectoryPos, trajectoryRot);

        /*
        if (mm.settings.databaseType == MMDatabaseType.Trajectory)
        mm.GetFeatureTrajectory(refPose, frameIdx, ref featureTrajectoryPos);*/
    }


    override public void OnDrawGizmos()
    {
        if (!mm.initialized) return;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(poseState.simulationPosition, poseState.simulationPosition + desiredVelocity * 2);
        drawTrajectory(trajectoryPos);
        Gizmos.color = Color.green;
        Vector3 motionVelocity = poseState.simulationRotation * poseState.motionVelocity;
        Vector3 pos = poseState.simulationPosition;
        Gizmos.DrawLine(pos, pos + motionVelocity * 10);
        if (prediction)
        {
            Gizmos.color = Color.red;
        }
        if (poseState!= null) poseState.Draw(settings.visScale);

        if (refPose != null) {
           drawRefPose();
        }
    }
    public void drawRefPose(){
        Gizmos.color = Color.yellow;
        refPose.Draw(settings.visScale);
        if (mm.settings.databaseType == MMDatabaseType.Trajectory){
            Gizmos.color = Color.red;
            mm.GetFeatureTrajectory(refPose, frameIdx, ref featureTrajectoryPos);
            drawTrajectory(featureTrajectoryPos);
            Gizmos.color = Color.red;
            var motionVelocity = refPose.motionRotation * refPose.motionVelocity;
            var pos = refPose.bonePositions[0];
            Gizmos.DrawLine(pos, pos + motionVelocity * 10);
        }
    }


    override public Vector3 UpdateDesiredVelocity(float dt)
    {
        Vector3 localDelta = Vector3.zero;
        Vector3 globalStickDir = Vector3.zero;
        if(target != null){

            var delta = target.position- transform.position;
            delta.y =0;
            float distance = delta.magnitude;
            if( distance < settings.minSpeed  && distance > 0.1){
                delta = delta.normalized * settings.minSpeed;
            }
            
            localDelta = Quaternion.Inverse(poseState.simulationRotation) * delta;
            globalStickDir = delta.normalized;
        }
        
        forwardSpeed = Mathf.Min(settings.speedFactor*Mathf.Abs(localDelta.z),settings.maxSpeed);
        sideSpeed = Mathf.Min(settings.speedFactor*Mathf.Abs(localDelta.x),settings.maxSpeed);
        
        var localStickDir = Quaternion.Inverse(poseState.simulationRotation) * globalStickDir;
        // Scale stick by forward, sideways and backwards speeds
        if(forwardSpeed >= sideSpeed) { 
            localStickDir *= forwardSpeed;
        }
        else
        {
            localStickDir *= sideSpeed;
        }
        return poseState.simulationRotation * localStickDir;
    }

    override public Quaternion UpdateDesiredRotation(Vector3 desiredVelocity, float dt)
    { 
        //var angles = Mathf.Deg2Rad * camera.PredictRotation(dt);
        if (Vector3.Magnitude(desiredVelocity ) > 0 && orientTowardsTarget){ 
            var direction = desiredVelocity.normalized;
            if (invertDirection) direction = -direction;
            var a = Mathf.Rad2Deg * Mathf.Atan2(direction.x, direction.z);
            return Quaternion.AngleAxis(a, new Vector3(0, 1, 0));
        }
        else if(target != null){ //copy target rotation

            //var cameraAngles = cameraController.PredictRotation(dt);
            //var cameraRot = Quaternion.AngleAxis(cameraAngles.y, new Vector3(0, 1, 0));
            var rotation = target.rotation;
            if (invertDirection) rotation *= Quaternion.Euler(0,180,0);
            return rotation;
        }else{
            return poseState.simulationRotation; // return current rotation
        }
        
    }



}

}