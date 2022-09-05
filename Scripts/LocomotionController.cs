using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Carousel
{

namespace MotionMatching{
    
public class LocomotionController : MMPoseProvider
{

    public PlayerInteractionZone interaction;
    protected PoseState refPose; 
    public MMControllerSettigs settings;
    public CameraController cameraController;
    public Vector3 stickDir;
    public bool prediction;
    public int frameIdx = 0;
    public float FPS = 60;
    public bool active = true;
    protected float syncTimer = 0;
    protected Vector3 desiredVelocity;
    public Quaternion desiredRotation;

    protected float forceSearchTimer = 0.1f;
    protected List<Vector3> trajectoryDesiredVel = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    protected List<Quaternion> trajectoryDesiredRot = new List<Quaternion>() { new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1) };

    protected List<Vector3> trajectoryPos = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    protected List<Vector3> trajectoryVel = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    protected List<Vector3> trajectoryAcc = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    protected List<Quaternion> trajectoryRot = new List<Quaternion>() { new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1) };
    protected List<Vector3> trajectoryAV = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };

    protected List<Vector3> featureTrajectoryPos = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    protected float forwardSpeed;
    protected float sideSpeed;
    public bool invertDirection;

    protected Quaternion _invertRotation = Quaternion.Euler(0,180,0);
    public bool useMotionVelocity;
    public bool useMotionAngularVelocity;
    public float velocityScale = 1f;
    public bool syncFPS;

    public List<int> annotationConstraint = new List<int>(){0, 0};
    public bool useAnnotationConstraint;

    [Range(0.0F, 1.0F)]
    public float spatialControlWeight = 1.0f;

    void Start()
    {
        var anim = GetComponent<Animator>();
        if(anim != null) anim.enabled = false;
        poseState = mm.Load();
        poseState.simulationPosition = transform.position;
        poseState.simulationPosition.y = 0;
        poseState.simulationRotation = transform.rotation;
        poseState.maxDegreesPerSecond = settings.maxDegreesPerSecond;
        poseState.useInterpolation = settings.useInterpolation;

        refPose = new PoseState(mm.database.nBones, mm.database.boneParents);
        mm.ComputeFeatures();

        annotationConstraint = new List<int>();
        for (int i = 0; i < mm.database.nAnnotations; i++)annotationConstraint.Add(0);
        syncTimer = 1.0f / FPS;
        prediction = false;
        frameIdx = settings.startFrameIdx;
        forceSearchTimer = settings.forceSearchTime;

    }
    void FixedUpdate(){
        if(syncFPS){
            syncTimer = 1.0f / FPS;
            int nSteps = (int)(Time.fixedDeltaTime/syncTimer);
            for(int i =0; i < nSteps; i++) Step(syncTimer);
        }
        else{
            Step(Time.fixedDeltaTime);
        }
        SetPose();
    
        refPose.SetState(mm.database, frameIdx, false);
    }

    virtual public void Step(float dt)
    {

        if (forceSearchTimer > 0)
        {
            forceSearchTimer -= dt;
        }
        var y = Input.GetAxis("Vertical");
        var x = Input.GetAxis("Horizontal");
        forwardSpeed = Mathf.Abs(y) * settings.maxSpeed;
        sideSpeed = Mathf.Abs(x) * settings.maxSpeed;
        stickDir = new Vector3(x, 0, y).normalized;

        desiredVelocity = UpdateDesiredVelocity(0);

        desiredRotation = UpdateDesiredRotation(desiredVelocity, 0);

        float predictionDt = dt * settings.predictionDistance;

        UpdateDesiredRotationTrajectory(predictionDt);
        UpdatePredictedRotationTrajectory(predictionDt);

        UpdateDesiredVelocityTrajectory(predictionDt);
        UpdatePredictedPositionTrajectory(predictionDt);


        //update simulation
        if(useMotionVelocity){
            var velocity = poseState.simulationRotation* poseState.motionVelocity*dt;
            
            poseState.simulationPosition += velocityScale * velocity;
          
        }else{
            simulationPositionsUpdate(ref poseState.simulationPosition, ref poseState.simulationVelocity, ref poseState.simulationAcceleration, desiredVelocity, settings.simulationVelocityHalflife, dt);
           
        }
        if(useMotionAngularVelocity){
            var av = poseState.simulationRotation* poseState.motionAV*dt;
            poseState.simulationRotation *= Quaternion.Euler(av.x*Mathf.Rad2Deg, av.y*Mathf.Rad2Deg, av.z*Mathf.Rad2Deg);
        }else{
            simulationRotationsUpdate(ref poseState.simulationRotation, ref poseState.simulationAV, desiredRotation, settings.simulationRotationHalflife, dt);
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

        frameIdx++;//prevents getting stuck
        verifyFrame();
        transform.position = poseState.simulationPosition;
        transform.rotation = poseState.simulationRotation;

    }
    
    protected void verifyFrame(){

        if(frameIdx >= mm.database.nFrames) {
            forceSearchTimer = 0;
            frameIdx = frameIdx-1;
        }
    }


    virtual public void FindTransition()
    {
        int oldFrameIdx = frameIdx;
        frameIdx = mm.FindTransition(poseState, frameIdx, trajectoryPos, trajectoryRot);

        if (mm.settings.databaseType == MMDatabaseType.Trajectory)
        mm.GetFeatureTrajectory(refPose, frameIdx, ref featureTrajectoryPos);
    }


    public void SetPose()
    {
        poseState.SetState(mm.database, frameIdx);
    }

    protected void drawTrajectory(List<Vector3> trajectory)
    {
        for (int i = 0; i < trajectory.Count; i++)
        {
            Gizmos.DrawSphere(trajectory[i], settings.visScale);
           // Gizmos.DrawMesh(PrimitiveType.Cube, trajectoryPos[i], trajectoryRot[i]);
        }
    }

  
    virtual public void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(poseState.simulationPosition, poseState.simulationPosition + desiredVelocity * 2);
        drawTrajectory(trajectoryPos);
        Gizmos.color = Color.red;
        drawTrajectory(featureTrajectoryPos);
        Gizmos.color = Color.green;
        if (prediction)
        {
            Gizmos.color = Color.red;
        }
        if (poseState!= null) poseState.Draw(settings.visScale);

        Gizmos.color = Color.yellow;
        if (refPose != null) refPose.Draw(settings.visScale);
    }

   

    virtual public Vector3 UpdateDesiredVelocity(float dt)
    {

        var cameraAngles = cameraController.PredictRotation(dt);
        var cameraRot = Quaternion.AngleAxis(cameraAngles.y, new Vector3(0, 1, 0));
        var globalStickDir = cameraRot * stickDir;
        var localStickDir = Quaternion.Inverse(poseState.simulationRotation) * globalStickDir;
        // Scale stick by forward, sideways and backwards speeds
        if(forwardSpeed >= sideSpeed) { 
            localStickDir *= forwardSpeed;
        }
        else
        {
            localStickDir *= sideSpeed;
        }
        /* localStickDir.x *= sideSpeed;
        localStickDir.z *= forwardSpeed;
          //Debug.Log(cameraAngles.y.ToString() + "_" + cameraRot.ToString()); globalStickDir;// 
         //Debug.Log(cameraAngles.y.ToString()+ stickDir.ToString() + "with rot "+ cameraRot.ToString() + " after " + globalStickDir.ToString());
         */
        return poseState.simulationRotation * localStickDir;
    }

    virtual public Quaternion UpdateDesiredRotation(Vector3 desiredVelocity, float dt)
    {
        Quaternion rotation;
        //var angles = Mathf.Deg2Rad * camera.PredictRotation(dt);
        if (Vector3.Magnitude(desiredVelocity ) > 0){ 
            var direction = desiredVelocity.normalized;
            var a = Mathf.Rad2Deg * Mathf.Atan2(direction.x, direction.z);
            rotation = Quaternion.AngleAxis(a, new Vector3(0, 1, 0));
        }
        else
        {
            var cameraAngles = cameraController.PredictRotation(dt);
            rotation = Quaternion.AngleAxis(cameraAngles.y, new Vector3(0, 1, 0));
        }
        if (invertDirection) rotation *= _invertRotation;
        
        return rotation;
    }

    protected void UpdateDesiredRotationTrajectory(float dt)
    {
        trajectoryDesiredRot[0] = poseState.simulationRotation;
        trajectoryAV[0] = poseState.simulationAV;
        for (int i = 1; i < trajectoryVel.Count; i++)
        {
            trajectoryDesiredRot[i] = UpdateDesiredRotation(desiredVelocity, dt*i);
        }
    }


    protected void UpdateDesiredVelocityTrajectory(float dt)
    {
        trajectoryDesiredVel[0] = desiredVelocity;
        for (int i = 1; i < trajectoryVel.Count; i++)
        {
            trajectoryDesiredVel[i] = UpdateDesiredVelocity(i*dt);
        }
    }

    protected void UpdatePredictedRotationTrajectory(float dt)
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

    protected void UpdatePredictedPositionTrajectory(float dt)
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


    protected void simulationRotationsUpdate(
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
    protected void simulationPositionsUpdate(
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
}