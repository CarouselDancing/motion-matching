using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Carousel.MotionMatching{
    
public class TargetLocomotionController : MMPoseProvider
{
    PoseState refPose; 
    public MMControllerSettigs settings;
    public bool prediction;
    public int frameIdx = 0;
    public float FPS = 60;
    public bool active = true;
    float syncTimer = 0;
    Vector3 desiredVelocity;
    public Quaternion desiredRotation;

    float forceSearchTimer = 0.1f;
    List<Vector3> trajectoryDesiredVel = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    List<Quaternion> trajectoryDesiredRot = new List<Quaternion>() { new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1) };

    List<Vector3> trajectoryPos = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    List<Vector3> trajectoryVel = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    List<Vector3> trajectoryAcc = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    List<Quaternion> trajectoryRot = new List<Quaternion>() { new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1) };
    List<Vector3> trajectoryAV = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };

    List<Vector3> featureTrajectoryPos = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    float forwardSpeed;
    float sideSpeed;
    public Transform target;
    public bool orientTowardsTarget;
    public bool useMotionVelocity;
    public bool invertDirection;

    Quaternion _invertRotation = Quaternion.Euler(0,180,0);

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


    void Update()
    {

        if (!active) return;
        float interval = 1.0f / FPS;
        if (syncTimer > 0) { 
            syncTimer -= Time.deltaTime;
            return;
        }
        syncTimer = interval;
        Step(interval);
    }

    void Step(float interval)
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
        if(useMotionVelocity){
            velocity = poseState.simulationRotation* poseState.motionVelocity;
        }
        Quaternion rotation = desiredRotation;
        simulationPositionsUpdate(ref poseState.simulationPosition, ref poseState.simulationVelocity, ref poseState.simulationAcceleration, velocity, settings.simulationVelocityHalflife, dt);
        simulationRotationsUpdate(ref poseState.simulationRotation, ref poseState.simulationAV, rotation, settings.simulationRotationHalflife, dt);

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

        if(frameIdx > mm.database.nFrames) {
            forceSearchTimer = 0;
            frameIdx = frameIdx-1;
        }

        transform.position = poseState.simulationPosition;
        transform.rotation = poseState.simulationRotation;

    }


    public void FindTransition()
    {
        int oldFrameIdx = frameIdx;
        frameIdx = mm.FindTransition(poseState, frameIdx, trajectoryPos, trajectoryRot);

        /*
        if (mm.settings.databaseType == MMDatabaseType.Trajectory)
        mm.GetFeatureTrajectory(refPose, frameIdx, ref featureTrajectoryPos);*/
    }


    public void SetPose()
    {
        poseState.SetState(mm.database, frameIdx);
    }

    void drawTrajectory(List<Vector3> trajectory)
    {
        for (int i = 0; i < trajectory.Count; i++)
        {
            Gizmos.DrawSphere(trajectory[i], settings.visScale);
           // Gizmos.DrawMesh(PrimitiveType.Cube, trajectoryPos[i], trajectoryRot[i]);
        }
    }

  
    public void OnDrawGizmos()
    {
        if (!mm.initialized) return;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(poseState.simulationPosition, poseState.simulationPosition + desiredVelocity * 2);
        drawTrajectory(trajectoryPos);
        Gizmos.color = Color.green;
        Vector3 motionVelocity = poseState.motionVelocity;
        Vector3 pos = poseState.simulationPosition;
        Gizmos.DrawLine(pos, pos + motionVelocity * 10);
        if (prediction)
        {
            Gizmos.color = Color.red;
        }
        if (poseState!= null) poseState.Draw(settings.visScale);

        Gizmos.color = Color.yellow;
        if (refPose != null) {
            refPose.Draw(settings.visScale);

            if (mm.settings.databaseType == MMDatabaseType.Trajectory){
                Gizmos.color = Color.red;
                mm.GetFeatureTrajectory(refPose, frameIdx, ref featureTrajectoryPos);
                drawTrajectory(featureTrajectoryPos);
                Gizmos.color = Color.red;
                motionVelocity = refPose.motionRotation * refPose.motionVelocity;
                pos = refPose.bonePositions[0];
                Gizmos.DrawLine(pos, pos + motionVelocity * 10);
            }
        }
    }


    Vector3 UpdateDesiredVelocity(float dt)
    {
        Vector3 localDelta = Vector3.zero;
        Vector3 globalStickDir = Vector3.zero;
        if(target != null){

            var delta = target.position- transform.position;
            delta.y =0;
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

    Quaternion UpdateDesiredRotation(Vector3 desiredVelocity, float dt)
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

    void UpdateDesiredRotationTrajectory(float dt)
    {
        trajectoryDesiredRot[0] = poseState.simulationRotation;
        trajectoryAV[0] = poseState.simulationAV;
        for (int i = 1; i < trajectoryVel.Count; i++)
        {
            trajectoryDesiredRot[i] = UpdateDesiredRotation(desiredVelocity, dt*i);
        }
    }
    
    void UpdateDesiredVelocityTrajectory(float dt)
    {
        trajectoryDesiredVel[0] = desiredVelocity;
        for (int i = 1; i < trajectoryVel.Count; i++)
        {
            trajectoryDesiredVel[i] = UpdateDesiredVelocity(i*dt);
        }
    }

    void UpdatePredictedRotationTrajectory(float dt)
    {
        for (int i = 0; i < trajectoryPos.Count; i++)
        {
            trajectoryRot[i] = poseState.simulationRotation;
            trajectoryAV[i] = poseState.simulationAV;
        }

        for (int i = 1; i < trajectoryPos.Count; i++)
        {
            simulationRotationsUpdate(ref trajectoryRot, ref trajectoryAV, trajectoryDesiredRot, i, settings.simulationVelocityHalflife, i * dt);
        }
    }

    void UpdatePredictedPositionTrajectory(float dt)
    {
        trajectoryPos[0] = poseState.simulationPosition;
        trajectoryVel[0] = poseState.simulationVelocity;
        trajectoryAcc[0] = poseState.simulationAcceleration;
        for (int i = 1; i < trajectoryPos.Count; i++)
        {
            trajectoryPos[i] = trajectoryPos[i - 1];
            trajectoryVel[i] = trajectoryVel[i - 1];
            trajectoryAcc[i] = trajectoryAcc[i - 1];

            simulationPositionsUpdate(ref trajectoryPos, ref trajectoryVel, ref trajectoryAcc, trajectoryDesiredVel, i, settings.simulationVelocityHalflife, dt);

        }
    }


    void simulationRotationsUpdate(
    ref List<Quaternion> rotations,
    ref List<Vector3> avs,
    List<Quaternion> desiredRotation,
    int index, float halflife, float dt)
    {
        float y = Utils.halflife_to_damping(halflife) / 2.0f;

        var inv_q = Quaternion.Inverse(desiredRotation[index]);

        var delta_q = rotations[index] * inv_q;
        if (delta_q.w < 0)
        {
            delta_q.w *= -1;
        }
        
        Vector3 j0 = Quat.to_scaled_angle_axis(delta_q);
        Vector3 j1 = avs[index] + j0 * y;

        float eydt = Utils.fast_negexpf(y * dt);

        rotations[index] = Quat.from_scaled_angle_axis(eydt * (j0 + j1 * dt)) * desiredRotation[index];
        avs[index] = eydt * (avs[index] - j1 * y * dt);
    }


    // Taken from https://theorangeduck.com/page/spring-roll-call#controllers
    void simulationPositionsUpdate(
        ref List<Vector3> position,
        ref List<Vector3> velocity,
        ref List<Vector3> acceleration,
        List<Vector3> desired_velocity,
        int index, float halflife,
        float dt)
    {
        float y = Utils.halflife_to_damping(halflife) / 2.0f;
        Vector3 j0 = velocity[index] - desired_velocity[index];
        Vector3 j1 = acceleration[index] + j0 * y;
        float eydt = Utils.fast_negexpf(y * dt);

        Vector3 position_prev = position[index];
        position[index] = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
            (j1 / (y * y)) + j0 / y + desired_velocity[index] * dt + position_prev;
        velocity[index] = eydt * (j0 + j1 * dt) + desired_velocity[index];
        acceleration[index] = eydt * (acceleration[index] - j1 * y * dt);
    }

  override public void ResetToIdle(){

  }


}

}