#include <assert.h>

#include "PxConfig.h"
#include "PxPhysicsAPI.h"

#define FRAME_TIME (1. / 30.)

using namespace physx;

PxFoundation* mFoundation = nullptr;
PxPhysics* mPhysics = nullptr;
PxCooking* mCooking = nullptr;
PxScene* mScene = nullptr;
PxMaterial* mMaterial = nullptr;
PxDefaultCpuDispatcher* disp;

PxPvd* pvd;
PxPvdTransport* transport;

PxDefaultAllocator allocator;
PxDefaultErrorCallback errorOut;

void PhysicsInit() {
	// Initialize foundation
	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION,
		allocator, errorOut);
	assert(mFoundation);

	pvd = PxCreatePvd(*mFoundation);
	assert(pvd);
	transport = PxDefaultPvdSocketTransportCreate("localhost", 5425, 10);
	assert(transport);
	pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	// Set up the tolerance scale
	PxTolerancesScale scale;
	scale.length = 1.f;
	scale.speed = 2.f / FRAME_TIME;
	assert(scale.isValid());

	// Initialize physics
	bool recordMemoryAllocations = true;
	mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation,
		scale, recordMemoryAllocations, pvd);
	assert(mPhysics);

	// Initialize cooking
	/*
	{
	PxCookingParams params(scale);
	// disable mesh cleaning - perform mesh validation on development configurations
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;
	// disable edge precompute, edges are set for each triangle, slows contact generation
	params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
	// lower hierarchy for internal mesh
	params.meshCookingHint = PxMeshCookingHint::eCOOKING_PERFORMANCE;

	mCooking->setParams(params);
	}
	//*/	
	mCooking = PxCreateCooking(PX_PHYSICS_VERSION,
		*mFoundation, PxCookingParams(scale));
	assert(mCooking);

	assert(PxInitExtensions(*mPhysics, pvd));

	// Make scene
	disp = PxDefaultCpuDispatcherCreate(1u);
	assert(disp);
	PxSceneDesc sceneDesc(scale);
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.cpuDispatcher = disp;
	sceneDesc.gravity = PxVec3(0.f, -0.4f, 0.f);
	assert(sceneDesc.isValid());
	mScene = mPhysics->createScene(sceneDesc);
	assert(mScene);

}

void PhysicsEnd() {
	mScene->release();
	disp->release();
	PxCloseExtensions();
	mCooking->release();
	mPhysics->release();
	pvd->release();
	transport->release();
	mFoundation->release();
}

void PhysicsTick(float dt) {
	mScene->simulate(dt);
	mScene->fetchResults(true);
}

void PhysicsCreateActors()
{
	// -------------------------------- Define the triangle mesh

	const PxU32 vces_size = 8;
	const PxVec3 vces[]{
		PxVec3(-1.f, 0.f, -1.f),
		PxVec3(-1.f, 0.f, 1.f),
		PxVec3(1.f, 0.f, -1.f),
		PxVec3(1.f, 0.f, 1.f),

		PxVec3(-2.f, 0.5f, -2.f),
		PxVec3(-2.f, 0.5f, 2.f),
		PxVec3(2.f, 0.5f, -2.f),
		PxVec3(2.f, 0.5f, 2.f)
	};
	const PxU32 ices_size = 30;
	const PxU32 ices[]{ 
		0u, 1u, 3u,
		0u, 3u, 2u,

		4u, 5u, 1u,
		4u, 1u, 0u,

		1u, 5u, 7u,
		1u, 7u, 3u,

		2u, 3u, 7u,
		2u, 7u, 6u,

		4u, 0u, 2u,
		4u, 2u, 6u
	};

	// -------------------------------- Create a material

	mMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.1f);

	// -------------------------------- Create a rigidbody

	PxRigidDynamic* rigidactor_ball = PxCreateDynamic(*mPhysics,
		PxTransform(PxVec3(1.5f, 3.f, 1.5f)),
		PxSphereGeometry(0.5f), *mMaterial, 0.2f,
		PxTransform(PxIdentity));

	rigidactor_ball->setLinearVelocity(PxVec3(0.f, -4.f, 0.f));

	// -------------------------------- Create a triangle mesh

	PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = vces_size;
	meshDesc.points.stride = sizeof(PxVec3);
	meshDesc.points.data = (void*)vces;
	meshDesc.triangles.count = ices_size / 3u;
	meshDesc.triangles.stride = 3u * sizeof(PxU32);
	meshDesc.triangles.data = (void*)ices;

	assert(meshDesc.isValid());

	//* live cooking
	PxTriangleMesh* aTriangleMesh = mCooking->createTriangleMesh(meshDesc,
		mPhysics->getPhysicsInsertionCallback());
	assert(aTriangleMesh);
	//*/

	/* How to make a cooked mesh for saving to a file or something
	PxDefaultMemoryOutputStream writeBuffer;
	PxTriangleMeshCookingResult::Enum result;
	bool status = mCooking->cookTriangleMesh(meshDesc, writeBuffer, &result);
	assert(status);
	PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
	PxTriangleMesh* aTriangleMesh = mPhysics->createTriangleMesh(readBuffer);
	//*/

	//* A simple method of creating the triangle mesh
	PxTriangleMeshGeometry geom(aTriangleMesh);
	assert(geom.isValid());
	PxRigidStatic* rigidactor_world = PxCreateStatic(*mPhysics, PxTransform(PxVec3(0.f, 0.f, 0.f)), geom, *mMaterial);
	assert(rigidactor_world);
	//*/

	/* An alternative method of creating the triangle mesh
	PxShape* shape_world =
	mPhysics->createShape(PxTriangleMeshGeometry(aTriangleMesh), *mMaterial, true);
	assert(shape_world);
	PxRigidStatic* rigidactor_world =
	mPhysics->createRigidStatic(PxTransform(PxVec3(0.f, 0.f, 0.f)));
	assert(rigidactor_world);
	rigidactor_world->attachShape(*shape_world);
	//*/

	// -------------------------------- Add the actors to the scene

	mScene->addActor(*rigidactor_ball);
	mScene->addActor(*rigidactor_world);
}

int main(char* argc, char** argv)
{
	PhysicsInit();
	PhysicsCreateActors();

	for (int i = 0; i < 512; ++i)
		PhysicsTick(FRAME_TIME);

	PhysicsEnd();

	return 0;
}
