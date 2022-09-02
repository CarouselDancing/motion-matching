using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public abstract class CustomTwoBoneIK : CharacterPoser
{
    public Transform root;
    public Transform elbow;
    public Transform end;
    public Transform target;
    public Transform hint;
    public Transform offsetPoint;
    Vector3 globalOffset;
    
    override public void SetTransforms()
    {
     
        initialized = true;
    }

    public Vector3 GetTargetPosition(){
        Vector3 tPosition = target.position;
        globalOffset = Vector3.zero;
        if(offsetPoint!=null){
            Vector3 localOffset = Quaternion.Inverse(end.rotation) * (offsetPoint.position-end.position);

            globalOffset = target.rotation * localOffset;
            tPosition = tPosition - globalOffset;
        }
        return tPosition;

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


