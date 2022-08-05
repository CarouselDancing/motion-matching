using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Carousel
{

namespace MotionMatching{
    
public class MotionPrediction : MonoBehaviour
{
    
    public float simulationVelocityHalflife = 0.27f;
    public float simulationRotationHalflife = 0.27f;
    public int predictionDistance = 20;
    public Vector3 simulationPosition;
    public Vector3 simulationVelocity;
    public Vector3 simulationAcceleration;
    public Quaternion simulationRotation;
    public Vector3 simulationAV;
    protected Vector3 desiredVelocity;
    public Quaternion desiredRotation;


    public float visScale = 0.1f;
    protected List<Vector3> trajectoryDesiredVel = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    protected List<Quaternion> trajectoryDesiredRot = new List<Quaternion>() { new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1) };

    protected List<Vector3> trajectoryPos = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    protected List<Vector3> trajectoryVel = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    protected List<Vector3> trajectoryAcc = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
    protected List<Quaternion> trajectoryRot = new List<Quaternion>() { new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1), new Quaternion(0, 0, 0, 1) };
    protected List<Vector3> trajectoryAV = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };


    Vector3 prevPos;
    void Start(){
        prevPos = transform.position;
        simulationPosition = prevPos;
        simulationRotation = transform.rotation;
    }

    // Update is called once per frame
    void Update()
    {
        updatePrediction(Time.deltaTime);
    }

    void updatePrediction(float dt){
        
        float predictionDt = dt * predictionDistance;
        desiredVelocity = (transform.position - prevPos) / dt;

        desiredRotation = transform.rotation;
        
        UpdateDesiredRotationTrajectory(predictionDt);
        UpdatePredictedRotationTrajectory(predictionDt);

        UpdateDesiredVelocityTrajectory(predictionDt);
        UpdatePredictedPositionTrajectory(predictionDt);

        simulationPositionsUpdate(ref simulationPosition, ref simulationVelocity, ref simulationAcceleration, desiredVelocity, simulationVelocityHalflife, dt);
        simulationRotationsUpdate(ref simulationRotation, ref simulationAV, desiredRotation, simulationRotationHalflife, dt);

        simulationPosition = transform.position;
        simulationRotation = transform.rotation;
        prevPos = transform.position;
    }
   

    public Vector3 GetPredictedPosition(){
        return trajectoryPos[3];
    }

    public Quaternion GetPredictedRotation(){
        return trajectoryRot[3];
    }

    protected void UpdateDesiredRotationTrajectory(float dt)
    {
        trajectoryDesiredRot[0] = simulationRotation;
        trajectoryAV[0] = simulationAV;
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
            trajectoryRot[i] = simulationRotation;
            trajectoryAV[i] = simulationAV;
        }

        for (int i = 1; i < trajectoryPos.Count; i++)
        {
            simulationRotationsUpdate(ref trajectoryRot, ref trajectoryAV, trajectoryDesiredRot, i, simulationVelocityHalflife, i * dt);
        }
    }

    protected void UpdatePredictedPositionTrajectory(float dt)
    {
        trajectoryPos[0] = simulationPosition;
        trajectoryVel[0] = simulationVelocity;
        trajectoryAcc[0] = simulationAcceleration;
        for (int i = 1; i < trajectoryPos.Count; i++)
        {
            trajectoryPos[i] = trajectoryPos[i - 1];
            trajectoryVel[i] = trajectoryVel[i - 1];
            trajectoryAcc[i] = trajectoryAcc[i - 1];

            simulationPositionsUpdate(ref trajectoryPos, ref trajectoryVel, ref trajectoryAcc, trajectoryDesiredVel, i, simulationVelocityHalflife, dt);

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




    public static void simulationRotationsUpdate(
    ref Quaternion rotations,
    ref Vector3 avs,
    Quaternion desiredRotation,
    float halflife, float dt)
    {
        float y = Utils.halflife_to_damping(halflife) / 2.0f;

        var inv_q = Quaternion.Inverse(desiredRotation);

        var delta_q = rotations * inv_q;
        if (delta_q.w < 0)
        {
            delta_q.w *= -1;
        }

        Vector3 j0 = Quat.to_scaled_angle_axis(delta_q);
        Vector3 j1 = avs + j0 * y;

        float eydt = Utils.fast_negexpf(y * dt);

        rotations = Quat.from_scaled_angle_axis(eydt * (j0 + j1 * dt)) * desiredRotation;
        avs = eydt * (avs - j1 * y * dt);
    }

    // Taken from https://theorangeduck.com/page/spring-roll-call#controllers

    public static void simulationPositionsUpdate(
    ref Vector3 position,
    ref Vector3 velocity,
    ref Vector3 acceleration,
    Vector3 desired_velocity,
   float halflife,
    float dt)
    {
        float y = Utils.halflife_to_damping(halflife) / 2.0f;
        Vector3 j0 = velocity - desired_velocity;
        Vector3 j1 = acceleration + j0 * y;
        float eydt = Utils.fast_negexpf(y * dt);

        Vector3 position_prev = position;
        position = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
            (j1 / (y * y)) + j0 / y + desired_velocity * dt + position_prev;
        velocity = eydt * (j0 + j1 * dt) + desired_velocity;
        acceleration = eydt * (acceleration - j1 * y * dt);
    }
    public static void simulationPositionsUpdateFromData(
    ref Vector3 position,
    ref Vector3 velocity,
    ref Vector3 acceleration,
    Vector3 desired_velocity,
   float halflife,
    float dt)
    {
        float y = Utils.halflife_to_damping(halflife) / 2.0f;
        Vector3 j0 = Vector3.zero;
        Vector3 j1 = acceleration + j0 * y;
        float eydt = Utils.fast_negexpf(y * dt);

        Vector3 position_prev = position;
        position = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
            (j1 / (y * y)) + j0 / y + desired_velocity * dt + position_prev;
        velocity = desired_velocity;
        acceleration = eydt * (acceleration - j1 * y * dt);
    }


      protected void drawTrajectory(List<Vector3> trajectory)
    {
        for (int i = 0; i < trajectory.Count; i++)
        {
            Gizmos.DrawSphere(trajectory[i], visScale);
        }
    }

  
    virtual public void OnDrawGizmos()
    {
        drawTrajectory(trajectoryPos);
    }

    virtual public Quaternion UpdateDesiredRotation(Vector3 desiredVelocity, float dt)
    {
        
        
        return transform.rotation;
    }

       virtual public Vector3 UpdateDesiredVelocity(float dt)
    {

        
        return desiredVelocity;
    }

}
}
}