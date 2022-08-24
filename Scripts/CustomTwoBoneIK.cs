using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class CustomTwoBoneIK : CharacterPoser
{
    public Transform root;
    public Transform elbow;
    public Transform end;
    public Transform target;
    public Transform hint;
    public Vector3 axis = new Vector3(1,0,0);
    public float limbAngle;
    public float lowerLimbLength;
    public float upperLimbLength;
    public float armLength;
    public float distanceToTarget;
    
    override public void SetTransforms()
    {
     
        initialized = true;
    }

    override public void UpdatePose()
    {
        if (target == null)return;
        upperLimbLength = (elbow.position - root.position).magnitude;
        lowerLimbLength = (end.position - elbow.position).magnitude;
        armLength = lowerLimbLength+upperLimbLength;
        var dirToTarget = target.position - root.position;
        distanceToTarget =  dirToTarget.magnitude;
        if ( distanceToTarget < armLength){
            limbAngle = 180-CalculateAngle(upperLimbLength, lowerLimbLength, distanceToTarget) *Mathf.Rad2Deg;
            var elbowQ = Quaternion.Euler(axis.x*limbAngle, axis.y*limbAngle,axis.z*limbAngle);
            elbow.localRotation = elbowQ.normalized;
        }else{
            elbow.localRotation = Quaternion.identity;

        }
        var dirToEnd = (end.position - root.position).normalized;
        dirToTarget = (target.position - root.position).normalized;

        var rootDelta = Quaternion.FromToRotation(dirToEnd, dirToTarget).normalized;
        var rotation =  Quaternion.Normalize(rootDelta*root.rotation);
        
        root.rotation = rotation;

        end.rotation = target.rotation;
        
    }

    float CalculateAngle(float a, float b, float c){
     /* get angle between a and b based on triangle lengths
        https://www.mathsisfun.com/algebra/trig-solving-sss-triangles.html */

        float cosC = (a*a + b*b - c*c) /(2*a*b);
        cosC = Mathf.Min(1, cosC);
        cosC = Mathf.Max(-1, cosC);
        float angle = Mathf.Acos(cosC);
        return angle;
    }

     // Update is called once per frame
    void FixedUpdate()
    {
        if(!active || inPipeline)return;
        //transform.position = src.GetGlobalPosition(Bones.Entity);
        if (!initialized) SetTransforms();
        UpdatePose();
    }
    
}


