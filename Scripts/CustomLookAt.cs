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
    public Vector3 positionnOffset = new Vector3(0,-0.3f,0);
    public Vector3 rotationOffset;

    void Start()
    {
        anim = GetComponent<Animator>();
        SetTransforms();
    }

    override public void SetTransforms()
    {
        boneTransform = anim.GetBoneTransform(bone);
        initialized = boneTransform != null;
    }


     override public void UpdatePose()
    {   
        if (targetTransform == null) return;
        var direction = (targetTransform.position) - boneTransform.position+positionnOffset;
        Quaternion desiredRotation = Quaternion.LookRotation(direction.normalized, Vector3.up);
        var offsetQ = Quaternion.Euler(rotationOffset.x, rotationOffset.y, rotationOffset.z);
        //var rotation = Quaternion.FromToRotation(m.dstUp, globalSrcUp).normalized;
        boneTransform.rotation = desiredRotation*offsetQ;

        
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