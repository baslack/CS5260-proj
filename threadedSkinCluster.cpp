/*
Benjamin A. Slack
Multi threaded skin cluster
CS5260

Basing this off the rudimentary cluster
coded by Autodesk as an example, I will
implement a multi-threaded version as an 
introduction to the API.

*/


//
//  File: threadedSkinCluster.cpp
//
//  Description:
//      Rudimentary implementation of a skin cluster.
//

#include <maya/MFnPlugin.h>
#include <maya/MTypeId.h> 

#include <maya/MMatrixArray.h>
#include <maya/MStringArray.h>

#include <maya/MPxSkinCluster.h> 
#include <maya/MItGeometry.h>
#include <maya/MPoint.h>
#include <maya/MFnMatrixData.h>

// adding my declarations

#include <maya/MThreadPool.h>
#include <maya/MGlobal.h>
#include <maya/MPointArray.h>
#include <maya/MPoint.h>
#include <maya/MProfiler.h>

#define NUM_TASK 16
#define NAME "threadedSkinCluster"

class threadedSkinCluster : public MPxSkinCluster
{
public:
    static  void*   creator();
    static  MStatus initialize();

    // Deformation function
    //
    virtual MStatus deform(MDataBlock&    block,
                           MItGeometry&   iter,
                           const MMatrix& mat,
                           unsigned int multiIndex);

    static const MTypeId id;
	static const int _profileCategory;
private:
	static MThreadRetVal skin(void *);
	static void decompose_skinning(void *, MThreadRootTask *);
	static const MPxNode::SchedulingType schedulingType();
};

const MTypeId threadedSkinCluster::id( 0x00080031);

const int threadedSkinCluster::_profileCategory(MProfiler::addCategory(NAME));

const MPxNode::SchedulingType threadedSkinCluster::schedulingType(){
	return MPxNode::kParallel;
}


void* threadedSkinCluster::creator()
{
    return new threadedSkinCluster();
}

MStatus threadedSkinCluster::initialize()
{
	// init the threadpool creation in the initialize
	MStatus stat = MThreadPool::init();
	if( MStatus::kSuccess != stat ) {
		MString str = MString("Error creating threadpool");
		MGlobal::displayError(str);
		return MStatus::kFailure;
	}
	return MStatus::kSuccess;
}

// wrapper struct for data that the threads need
typedef struct _thread_data{
	int threadNo;
	long start_index;
	long end_index;
	int num_tm;
	MMatrixArray* tm;
	MPointArray* pts;
	MDataBlock* block;
}thread_data;

//wrapper struct for data that the thread dispatcher needs
typedef struct _task_data{
	long num_points;
	int num_tm;
	MMatrixArray* tm;
	MPointArray* pts;
	MDataBlock* block;
	MItGeometry* iter;
}task_data;

MThreadRetVal threadedSkinCluster::skin(void *data){
	thread_data *my_data = (thread_data *)data;
	MMatrixArray tm = *my_data->tm;
	MPointArray *pts = my_data->pts;
	MDataBlock block = *my_data->block;

	// there's no go way to get hold of the weights en masse, so 
	// a new handle seems like the way to go

	MArrayDataHandle weight_list_handle = block.inputArrayValue(weightList);
	weight_list_handle.jumpToArrayElement(my_data->start_index);

	int skin_event =  MProfiler::eventBegin(threadedSkinCluster::_profileCategory, MProfiler::kColorC_L2, "skin: loop");

	for (long index = my_data->start_index; index <= my_data->end_index; index++){
		MPoint skinned;
		//get the weights for this index
		MArrayDataHandle weights_handle = weight_list_handle.inputValue().child(weights);
		
		//calculate the skinned points
		for (int i = 0; i < my_data->num_tm; i++){
			//get the weight for each tm
			if (MS::kSuccess == weights_handle.jumpToElement(i)){
				skinned += ((*pts)[index] * tm[i]) * weights_handle.inputValue().asDouble();
			}
		}

		//set the point to the new position
		(*pts)[index]=skinned;

		//get next set of weights
		weight_list_handle.next();
	}

	MProfiler::eventEnd(skin_event);

	return (MThreadRetVal)0;
}

void threadedSkinCluster::decompose_skinning(void *data, MThreadRootTask *root){

	task_data *my_data = (task_data *)data;

	MItGeometry *iter = my_data->iter;
	MPointArray *pts = my_data->pts;

	thread_data tdata[NUM_TASK];

	long start_index, end_index;
	start_index = 0;
	end_index = 0;
	long interval = my_data->num_points/NUM_TASK;
	long remainder = my_data->num_points%NUM_TASK;
	
	int dispatch_event = MProfiler::eventBegin(threadedSkinCluster::_profileCategory, MProfiler::kColorC_L1, "decompose: loop");
	for (int i = 0; i < NUM_TASK; i++){
		//setup thread data
		//calc starting indexes
		if ((end_index != 0) || (i > 0)){
			start_index = end_index + 1;
		}
		end_index = start_index + interval - 1;
		if (i <= remainder){
			end_index++;
		}
		//populate thread data struct
		tdata[i].start_index = start_index;
		tdata[i].end_index = end_index;
		tdata[i].block = my_data->block;
		tdata[i].num_tm = my_data->num_tm;
		tdata[i].tm = my_data->tm;
		tdata[i].pts = my_data->pts;
		tdata[i].threadNo = i;
		//spawn a thread
		MThreadPool::createTask(threadedSkinCluster::skin, (void *)&tdata[i], root);
	}
	MProfiler::eventEnd(dispatch_event);

	int thread_fire_event = MProfiler::eventBegin(threadedSkinCluster::_profileCategory, MProfiler::kColorC_L3, "decompose: exec");

	// fire off the threads
	MThreadPool::executeAndJoin(root);

	MProfiler::eventEnd(thread_fire_event);

	// set the geo positions
	(*iter).setAllPositions((*pts));
}



MStatus
threadedSkinCluster::deform( MDataBlock& block,
                      MItGeometry& iter,
                      const MMatrix& /*m*/,
                      unsigned int multiIndex)
//
// Method: deform
//
// Description:   Deforms the point with a simple smooth skinning algorithm
//
// Arguments:
//   block      : the datablock of the node
//   iter       : an iterator for the geometry to be deformed
//   m          : matrix to transform the point into world space
//   multiIndex : the index of the geometry that we are deforming
//
//
{
    MStatus returnStatus;
    
	// get the influence transforms
	//
	// Maya API interacts with Datablocks using "handles"
	// ArrayDataHandle grants access to node's array data
	// elements
	// in this case, the TM handle being pulled from the block is
	// getting the skin cluster's matrix attr/plug passed to it
	MArrayDataHandle transformsHandle = block.inputArrayValue( matrix );
	// note: this is required to "prime the pump" on the handle
	// without it, later calls to the handle's methods will fail
	int numTransforms = transformsHandle.elementCount();
	
	// if there's no transforms, we're done deforming
	if ( numTransforms == 0 ) {
		return MS::kSuccess;
	}
	

	// allocates space for an array of TMs
	MMatrixArray transforms;
	// populates the array pulling data from the handle block
	for ( int i=0; i<numTransforms; ++i ) {
		//breaking this down
		//transformsHandle.inputValue().data(), this is getting the actual matrix data off the plug
		//inputValue is returning a MDataHandle. The .data() method then gets the MObject Wrapper
		//there's a lot of indirection in the Maya API apparently
		//MFnMatrixData(tmhandle.input.data).matrix(), takes the MObject and attaches MMatrix
		//functions to it, data method of tmHandle is returning a MObject wrapper
		//MMatrix, .matrix() this actually gives the 4x4 TM.
		transforms.append( MFnMatrixData( transformsHandle.inputValue().data() ).matrix() );
		transformsHandle.next();
	}
	
	//same story as the previous, first we get the handle
	//the count is already initialized
	//this time we're getting the bindPreMatrix attr of the cluster
	MArrayDataHandle bindHandle = block.inputArrayValue( bindPreMatrix );
	if ( bindHandle.elementCount() > 0 ) {
		for ( int i=0; i<numTransforms; ++i ) {
			//breaking it down
			//using the attr handle, we get a Data Handle, then get the MObject off that handle
			//next we put the matrix functions on that MObject
			//next we pre-multiply the pre-bind matrix to the existing tm and store it back on itself
			transforms[i] = MFnMatrixData( bindHandle.inputValue().data() ).matrix() * transforms[i];
			bindHandle.next();
		}
	}

	//pulls the weights off the cluster
	MArrayDataHandle weightListHandle = block.inputArrayValue( weightList );
	if ( weightListHandle.elementCount() == 0 ) {
		// no weights - nothing to do
		return MS::kSuccess;
	}


	//the deform method gets an geo iterator passed in
	//to thread this, that iterator is going to need
	//to be broken up, somehow
	//inital investigation points at the "all positions" method
	//this loop would then need to be replaced with the thread
	//pool tasks mapping

	// this should take care of getting the points
	MPointArray my_points = MPointArray();
	MStatus stat = iter.allPositions(my_points);
	long num_pts = my_points.length();

	// prep data for task decomposition
	task_data tdata;
	tdata.block = &block;
	tdata.num_points = num_pts;
	tdata.pts = &my_points;
	tdata.num_tm = numTransforms;
	tdata.iter = &iter;
	tdata.tm = &transforms;
	
	//fire task decomposition
	MThreadPool::newParallelRegion(threadedSkinCluster::decompose_skinning, (void *)&tdata);

    return returnStatus;
}


// standard initialization procedures
//

MStatus initializePlugin( MObject obj )
{
    MStatus result;

    // ids the plugin to the system in maya
	MFnPlugin plugin( obj, "Benjamin Slack", "0.1", "Any");
    result = plugin.registerNode(
        "threadedSkinCluster" ,
        threadedSkinCluster::id ,
        &threadedSkinCluster::creator ,
        &threadedSkinCluster::initialize ,
        MPxNode::kSkinCluster
        );

	// this is where the threadpool needs to be setup

    return result;
}

MStatus uninitializePlugin( MObject obj )
{
    MStatus result;

    MFnPlugin plugin( obj );
    result = plugin.deregisterNode( threadedSkinCluster::id );

	// this is where the threadpool needs to be released
	MThreadPool::release();

    return result;
}
