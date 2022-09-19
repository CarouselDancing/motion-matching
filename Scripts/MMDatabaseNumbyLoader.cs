using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Diagnostics;
using NumSharp;


namespace Carousel.MotionMatching{

class MMDatabaseNumbyLoader{
    public MMSettings settings;

    public MMDatabaseNumbyLoader(MMSettings settings){
        this.settings = settings;
    }

    

    public MMDatabase LoadResource(string fileName){
        TextAsset asset = Resources.Load(fileName) as TextAsset;
        Stream stream = new MemoryStream(asset.bytes);
        return LoadFromFileStream(stream);

    }

    public MMDatabase Load(string filename){
        using (var stream = File.Open(filename, FileMode.Open, FileAccess.Read, FileShare.Read ))
        {
            return LoadFromFileStream(stream);
        }
           
          
    }

    public MMDatabase LoadFromFileStream(Stream stream){
        
        var db = new MMDatabase(settings);
        NpzDictionary<Array> data;
        try { 
            //data = np.Load_Npz<Array>(filename);
            data = new NpzDictionary<Array>(stream);
        }
        catch (Exception e)
        {
            UnityEngine.Debug.Log("Cant load file");
            UnityEngine.Debug.Log(e.Message);
        
            return db;

        }
        var metaDataKeysArray = (int[]) data["meta_data_keys.npy"];
        List<string> metaDataKeys = IntToStringList(metaDataKeysArray);
        float[]  metaDataValues =(float[])  data["meta_data_values.npy"];
        int nFramesIndex = metaDataKeys.FindIndex(v => v == "nFrames"); 
        int nBonesIndex = metaDataKeys.FindIndex(v => v == "nBones");
        int fpsIndex =  metaDataKeys.FindIndex(v => v == "fps"); 
        db.nFrames = (int) metaDataValues[nFramesIndex];
        db.nBones = (int)  metaDataValues[nBonesIndex];
        db.fps = metaDataValues[fpsIndex];
        /*
        int[] nFramesArray = (int[]) data["nFrames.npy"];
        nFrames = nFramesArray[0];
        int[] nBonesArray = (int[]) data["nBones.npy"];
        nBones = nBonesArray[0];

        float[] fpsArray = (float[]) data["fps.npy"];
        fps = fpsArray[0];
        */


        float[,,] positions = (float[,,]) data["bone_positions.npy"];
        float[,,] velocities = (float[,,]) data["bone_velocities.npy"];
        float[,,]  rotations =(float[,,])  data["bone_rotations.npy"];
        float[,,]  angularVelocities =(float[,,])  data["bone_angular_velocities.npy"];
        db.bonePositions = new Vector3[db.nFrames,db.nBones];
        db.boneVelocities = new Vector3[db.nFrames,db.nBones];
        db.boneRotations = new Quaternion[db.nFrames,db.nBones];
        db.boneAngularVelocities = new Vector3[db.nFrames,db.nBones];
        for (int i = 0; i < db.nFrames; i++)
        {
            for (int j = 0; j < db.nBones; j++)
            {
                db.bonePositions[i, j].x = -positions[i,j,0];
                db.bonePositions[i, j].y = positions[i,j,1];
                db.bonePositions[i, j].z = positions[i,j,2];


                db.boneRotations[i, j].w = -rotations[i,j,0];
                db.boneRotations[i, j].x = -rotations[i,j,1];
                db.boneRotations[i, j].y = rotations[i,j,2];
                db.boneRotations[i, j].z = rotations[i,j,3];


                db.boneVelocities[i, j].x = -velocities[i,j,0];
                db.boneVelocities[i, j].y = velocities[i,j,1];
                db.boneVelocities[i, j].z = velocities[i,j,2];


                db.boneAngularVelocities[i, j].x = -angularVelocities[i,j,0];
                db.boneAngularVelocities[i, j].y = angularVelocities[i,j,1];
                db.boneAngularVelocities[i, j].z = angularVelocities[i,j,2];



            }
        }
        try{
            var intContactStates = (int[,]) data["contact_states.npy"];
            db.contactStates = new bool[db.nFrames,2];
            for (int i = 0; i < db.nFrames; i++)
            {
                
                    db.contactStates[i, 0] = intContactStates[i,0]==0;
                    db.contactStates[i, 1] = intContactStates[i,1]==1;
            }
        }catch{

        }

        if(data.ContainsKey("phase_data.npy")){
            db.phaseData = (float[]) data["phase_data.npy"];
            
        }

        db.boneParents =(int[])  data["bone_parents.npy"];
        db.rangeStart =(int[])  data["range_starts.npy"];
        db.rangeStop =(int[])  data["range_stops.npy"];

        var boneMapArray =(int[])  data["bone_map.npy"];
        var boneNamesArray = (int[]) data["bone_names.npy"];
        db.boneNames = IntToStringList(boneNamesArray);
        db.boneArray = new List<HumanBodyBones>();
        db.boneIndexMap = new Dictionary<HumanBodyBones, int>();
        for (int i = 0; i < db.boneNames.Count; i++)
        {
            var bone = (HumanBodyBones)boneMapArray[i];
            db.boneArray.Add(bone);
            db.boneMap[bone] = db.boneNames[i];
            db.boneIndexMap[bone] = i;
        }

            
        if(data.ContainsKey("annotation_keys.npy")&& data.ContainsKey("annotation_values.npy") && data.ContainsKey("annotation_matrix.npy")){
            var annotationKeysArray = (int[]) data["annotation_keys.npy"];
            db.annotationKeys = IntToStringList(annotationKeysArray);
            db.nAnnotations= db.annotationKeys.Count;
            var annotationValuesArray = (int[]) data["annotation_values.npy"];
            db.annotationValues = IntToStringList(annotationValuesArray);
            db.annotationMatrix =(int[,])  data["annotation_matrix.npy"];
            UnityEngine.Debug.Log("error"+db.annotationMatrix.GetLength(0).ToString() +"_"+ db.annotationMatrix.GetLength(1).ToString() );
           
        }

        UnityEngine.Debug.Log("finished loading");
        return db;

    }


    List<string> IntToStringList(int[] intArray){
        string nameStr = "";
        for (int i =0; i < intArray.Length; i++){
            nameStr += (char)intArray[i];

        }
        return new List<string>(nameStr.Split(','));
    }

}
}