using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomLookAt : CharacterPoser
{
    
    Animator anim;
    public HumanBodyBones bone = HumanBodyBones.Neck;
    public Transform boneTransform;
    public HumanBodyBones targetBone = HumanBodyBones.Head;
    public Transform targetTransform;
    public Vector3 positionOffset = new Vector3(0,-0.3f,0);
    public Vector3 rotationOffset;
    public Vector3 localDesiredRotationE;
    public Quaternion defaultRotation;
    public Vector3 maxAngles = new Vector3(90,90,90);
    public bool applyLimits= true;

    void Start()
    {
        anim = GetComponent<Animator>();
        SetTransforms();
    }

    override public void SetTransforms()
    {
        boneTransform = anim.GetBoneTransform(bone);
        initialized = boneTransform != null;
        if(initialized) defaultRotation = boneTransform.localRotation;
    }

    public float ClampAngle(float angle, float maxAngle){
            if(Mathf.Abs(angle) > 180) {
                if(angle < 0){
                    angle = (360+angle) % 360;
                }else{
                    angle = -(360-angle) % 360;
                }
            }
            angle = Mathf.Clamp(angle, -maxAngle, maxAngle);
            return angle;
    }


     override public void UpdatePose()
    {   
        if (targetTransform == null) return;
        var direction = (targetTransform.position) - boneTransform.position+positionOffset;
        Quaternion desiredRotation = Quaternion.LookRotation(direction.normalized, Vector3.up);
        if (applyLimits){
            var localDesiredRotation  = Quaternion.Inverse(boneTransform.parent.rotation)*Quaternion.Inverse(defaultRotation)*  desiredRotation;
            localDesiredRotationE = localDesiredRotation.eulerAngles;
            localDesiredRotationE.x = ClampAngle(localDesiredRotationE.x, maxAngles.x);
            localDesiredRotationE.y = ClampAngle(localDesiredRotationE.y, maxAngles.y);
            localDesiredRotationE.z = ClampAngle(localDesiredRotationE.z, maxAngles.z);
            localDesiredRotation = Quaternion.Euler(localDesiredRotationE.x, localDesiredRotationE.y, localDesiredRotationE.z);
            desiredRotation = boneTransform.parent.rotation * defaultRotation * localDesiredRotation;
        }
        var offsetQ = Quaternion.Euler(rotationOffset.x, rotationOffset.y, rotationOffset.z);
        boneTransform.rotation = desiredRotation*offsetQ;

        
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if(!active || inPipeline)return;
        if (!initialized) SetTransforms();
        UpdatePose();
    }
}