using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PoseCompositor : MonoBehaviour
{
    public List<CharacterPoser> posers;


   // Update is called once per frame
    void FixedUpdate()
    {
      UpdatePose();
    }

    public void UpdatePose(){
        foreach(var poser in posers){
            if(!poser.initialized)poser.SetTransforms();
            if(poser.active) poser.UpdatePose();
        }
    }

    public void Add(CharacterPoser poser){
        poser.inPipeline = true;
        posers.Add(poser);
    }
}