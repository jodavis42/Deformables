#include "SpringsPrecompiled.hpp"

//-----------------------------------------------------------------------------ClothSpringSystemEvent
ZilchDefineType(ClothSpringSystemEvent, builder, type)
{
  // This is required for event binding
  ZilchBindDestructor();
  ZilchBindConstructor();

  ZilchBindFieldProperty(mClothSystem);
  ZilchBindFieldProperty(mDt);
}

//-----------------------------------------------------------------------------ClothSpringSystem
ZilchDefineType(ClothSpringSystem, builder, type)
{
  // This is required for component binding
  ZilchBindDestructor();
  ZilchBindConstructor();
  ZilchBindMethod(Initialize);

  // Note: All event connection methods must be bound
  ZilchBindMethod(OnLogicUpdate);

  ZilchBindGetter(ParticleCount);
  ZilchBindMethod(AddParticle);
  ZilchBindMethod(GetParticlePosition);
  ZilchBindMethod(SetParticlePosition);
  ZilchBindMethod(GetParticleInvMass);
  ZilchBindMethod(SetParticleInvMass);
  ZilchBindMethod(ApplyParticleForce);

  ZilchBindGetter(SpringCount);
  ZilchBindMethod(AddSpring);
  ZilchBindMethod(GetRestLength);
  ZilchBindMethod(SetRestLength);

  ZilchBindGetter(FaceCount);
  ZilchBindMethod(AddFace);
  ZilchBindMethod(GetFaceIndices);
  ZilchBindMethod(GetFaceIndexA);
  ZilchBindMethod(GetFaceIndexB);
  ZilchBindMethod(GetFaceIndexC);
  ZilchBindMethod(AddAnchor);

  ZilchBindMethod(UploadToMesh);
  ZilchBindMethod(SetMesh);
  ZilchBindMethod(AddUv);
  ZilchBindMethod(ClearSystem);

  ZilchBindFieldProperty(mSubDivisions);
  ZilchBindFieldProperty(mIterations);
  ZilchBindFieldProperty(mIntegrationMethod);
  ZilchBindFieldProperty(mUploadInLocalSpace);
}

ClothSpringSystem::ClothSpringSystem()
{
  mSubDivisions = 1;
  mIterations = 1;
  mMesh = nullptr;
  mUploadInLocalSpace = true;
  mIntegrationMethod = IntegrationMethod::SemiImplicitEuler;
}

ClothSpringSystem::~ClothSpringSystem()
{

}

void ClothSpringSystem::Initialize(ZeroEngine::CogInitializer* initializer)
{
  ZeroConnectThisTo(this->GetSpace(), "LogicUpdate", "OnLogicUpdate");
}

void ClothSpringSystem::OnLogicUpdate(ZeroEngine::UpdateEvent* event)
{
  Real dt = event->GetDt() / mSubDivisions;
  for (int i = 0; i < mSubDivisions; ++i)
  {
    if (mIntegrationMethod == IntegrationMethod::Jakobsen)
      IterateJakobsenTimestep(dt);
    else
      IterateBasicTimestep(dt);
  }

  UploadToMesh(mMesh);
}

void ClothSpringSystem::IterateBasicTimestep(Real dt)
{
  CalcuateGlobalForces(dt);
  CalculateSpringForces(dt);
  Integrate(dt);
  ClearForces();
}

void ClothSpringSystem::IterateJakobsenTimestep(Real dt)
{
  UpdateAnchors(dt);
  CalcuateGlobalForces(dt);
  IntegrateVerlet(dt);
  for(size_t i = 0; i < (size_t)mIterations; ++i)
    SatisfyConstraintsJakobsen(dt);
  ClearForces();
}

void ClothSpringSystem::UpdateAnchors(Real dt)
{
  for (size_t i = 0; i < mAnchors.Size(); ++i)
  {
    ClothAnchor& anchor = mAnchors[i];
    int index = anchor.mIndex;

    Cog* anchorCog = anchor.mAnchor;
    if (anchorCog == nullptr)
      continue;
    Transform* transform = anchorCog->has(Transform);
    if (transform == nullptr)
      continue;
    Real3 worldPoint = transform->TransformPoint(anchor.mLocalPosition);
    mPositions[index] = worldPoint;
    mVelocities[index] = mPositions[index] - mOldPositions[index];
  }
}

void ClothSpringSystem::CalcuateGlobalForces(Real dt)
{
  Zilch::HandleOf<ClothSpringSystemEvent> toSend = ZilchAllocate(ClothSpringSystemEvent);
  toSend->mClothSystem = this;
  toSend->mDt = dt;
  this->GetOwner()->DispatchEvent("ComputeClothForces", toSend);
  this->GetSpace()->DispatchEvent("ComputeClothForces", toSend);
}

void ClothSpringSystem::CalculateSpringForces(Real dt)
{
  for(size_t i = 0; i < mSprings.Size(); ++i)
  {
    ClothSpring& spring = mSprings[i];
    Real3 posA = mPositions[spring.mIndexA];
    Real3 posB = mPositions[spring.mIndexB];
    Real3 velA = mVelocities[spring.mIndexA];
    Real3 velB = mVelocities[spring.mIndexB];

    Real3 deltaPosition = posB - posA;
    Real3 deltaVelocity = velB - velA;

    Real distance = Math::Length(deltaPosition);
    Real3 direction = Math::Normalized(deltaPosition);

    Real displacement = spring.mRestLength - distance;
    Real3 forceSpring = -displacement * direction * spring.mStiffness;
    Real3 forceDrag = spring.mDamping * Math::Dot(deltaVelocity, direction) * direction;

    Real3 force = forceSpring + forceDrag;

    mForces[spring.mIndexA] += force;
    mForces[spring.mIndexB] -= force;
  }
}

void ClothSpringSystem::SatisfyConstraintsJakobsen(Real dt)
{
  for (size_t i = 0; i < mSprings.Size(); ++i)
  {
    ClothSpring& spring = mSprings[i];
    Real invMassA = mInvMasses[spring.mIndexA];
    Real invMassB = mInvMasses[spring.mIndexB];
    Real3 posA = mPositions[spring.mIndexA];
    Real3 posB = mPositions[spring.mIndexB];
    Real3 velA = mVelocities[spring.mIndexA];
    Real3 velB = mVelocities[spring.mIndexB];

    Real3 deltaPosition = posB - posA;
    Real3 deltaVelocity = velB - velA;
    Real displacement = Math::Length(deltaPosition);
    Real denom = displacement * (invMassA + invMassB);
    Real numer = displacement - spring.mRestLength;
    if (denom == 0)
      continue;

    Real3 impulse = -deltaPosition * (numer / denom) * spring.mStiffness;
    mPositions[spring.mIndexA] -= invMassA * impulse;
    mPositions[spring.mIndexB] += invMassB * impulse;
  }
}

void ClothSpringSystem::Integrate(Real dt)
{
  if (mIntegrationMethod == IntegrationMethod::SemiImplicitEuler)
    IntegrateEuler(dt);
  else
    IntegrateVerlet(dt);
}

void ClothSpringSystem::IntegrateEuler(Real dt)
{
  Real3 gravity = -Real3(0, 10, 0);

  for (size_t i = 0; i < mPositions.Size(); ++i)
  {
    Real invMass = mInvMasses[i];

    if (invMass == 0)
      continue;

    Real3 acceleration = mForces[i] * invMass + gravity;
    
    mVelocities[i] += dt * acceleration;
    mOldPositions[i] = mPositions[i];
    mPositions[i] += dt * mVelocities[i];
  }
}

void ClothSpringSystem::IntegrateVerlet(Real dt)
{
  Real3 gravity = -Real3(0, 10, 0);

  for (size_t i = 0; i < mPositions.Size(); ++i)
  {
    Real invMass = mInvMasses[i];

    if (invMass == 0)
      continue;

    Real3 force = mForces[i];
    Real3 position = mPositions[i];
    Real3 oldPosition = mOldPositions[i];

    Real3 acceleration = force * invMass + gravity;

    Real3 newPosition =  2 * position - oldPosition + acceleration * dt * dt;

    mPositions[i] = newPosition;
    mOldPositions[i] = position;
    mVelocities[i] = newPosition - position;
    mVelocities[i] += acceleration * dt;
  }
}

void ClothSpringSystem::ClearForces()
{
  for (size_t i = 0; i < mPositions.Size(); ++i)
    mForces[i].ZeroOut();
}

void ClothSpringSystem::UploadToMesh(ZeroEngine::Mesh* mesh)
{
  if (mesh == nullptr)
    return;

  Array<Real3> normals;
  normals.Resize(mPositions.Size(), Real3::cZero);
  for (size_t i = 0; i < mFaces.Size(); ++i)
  {
    ClothFace& face = mFaces[i];
    Real3& posA = mPositions[face.mIndexA];
    Real3& posB = mPositions[face.mIndexB];
    Real3& posC = mPositions[face.mIndexC];
    Real3 normal = Math::Normalized(Math::Cross(posB - posA, posC - posA));

    normals[face.mIndexA] += normal;
    normals[face.mIndexB] += normal;
    normals[face.mIndexC] += normal;
  }

  if (mUploadInLocalSpace)
    UploadToMeshLocal(mesh, normals);
  else
    UploadToMeshWorld(mesh, normals);

  mesh->Upload();
}

void ClothSpringSystem::UploadToMeshWorld(ZeroEngine::Mesh* mesh, Array<Real3>& normals)
{
  auto vertices = mesh->GetVertices();
  vertices->ClearData();
  for (size_t i = 0; i < mPositions.Size(); ++i)
  {
    vertices->AddReal(mPositions[i]);
    vertices->AddReal(mUvs[i]);
    vertices->AddReal(normals[i]);
  }
}

void ClothSpringSystem::UploadToMeshLocal(ZeroEngine::Mesh* mesh, Array<Real3>& normals)
{
  Transform* transform = GetOwner()->has(Transform);
  Real4x4 localToWorld = transform->GetWorldMatrix();
  Real4x4 worldToLocal = localToWorld.Inverted();

  auto vertices = mesh->GetVertices();
  vertices->ClearData();
  for (size_t i = 0; i < mPositions.Size(); ++i)
  {
    Real3 localPosition = Math::TransformPoint(worldToLocal, mPositions[i]);
    vertices->AddReal(localPosition);
    vertices->AddReal(mUvs[i]);
    vertices->AddReal(normals[i]);
  }
}

int ClothSpringSystem::GetParticleCount()
{
  return mPositions.Size();
}

void ClothSpringSystem::AddParticle(Real3Param position, Real invMass)
{
  mPositions.PushBack(position);
  mOldPositions.PushBack(position);
  mInvMasses.PushBack(invMass);
  mVelocities.PushBack(Real3::cZero);
  mForces.PushBack(Real3::cZero);
}

Real3 ClothSpringSystem::GetParticlePosition(int index)
{
  return mPositions[index];
}

void ClothSpringSystem::SetParticlePosition(int index, Real3Param position)
{
  mPositions[index] = position;
}

Real ClothSpringSystem::GetParticleInvMass(int index)
{
  return mInvMasses[index];
}

void ClothSpringSystem::SetParticleInvMass(int index, Real invMass)
{
  mInvMasses[index] = invMass;
}

void ClothSpringSystem::ApplyParticleForce(int index, Real3Param force)
{
  mForces[index] += force;
}

int ClothSpringSystem::GetSpringCount()
{
  return mSprings.Size();
}

void ClothSpringSystem::AddSpring(int indexA, int indexB, Real stiffness, Real damping)
{
  ClothSpring spring;
  spring.mIndexA = indexA;
  spring.mIndexB = indexB;
  spring.mStiffness = stiffness;
  spring.mDamping = damping;

  Real3 posA = mPositions[indexA];
  Real3 posB = mPositions[indexB];
  spring.mRestLength = Math::Distance(posA, posB);
  mSprings.PushBack(spring);
}

Real ClothSpringSystem::GetRestLength(int index)
{
  return mSprings[index].mRestLength;
}

void ClothSpringSystem::SetRestLength(int index, Real restLength)
{
  mSprings[index].mRestLength = restLength;
}

int ClothSpringSystem::GetFaceCount()
{
  return mFaces.Size();
}

void ClothSpringSystem::AddFace(int indexA, int indexB, int indexC)
{
  ClothFace face;
  face.mIndexA = indexA;
  face.mIndexB = indexB;
  face.mIndexC = indexC;
  mFaces.PushBack(face);
}

Integer3 ClothSpringSystem::GetFaceIndices(int index)
{
  ClothFace& face = mFaces[index];
  return Integer3(face.mIndexA, face.mIndexB, face.mIndexC);
}

int ClothSpringSystem::GetFaceIndexA(int index)
{
  return mFaces[index].mIndexA;
}

int ClothSpringSystem::GetFaceIndexB(int index)
{
  return mFaces[index].mIndexB;
}

int ClothSpringSystem::GetFaceIndexC(int index)
{
  return mFaces[index].mIndexC;
}

void ClothSpringSystem::AddAnchor(int index, Cog* anchorCog)
{
  if (anchorCog == nullptr)
    return;

  Real3 position = mPositions[index];
  Transform* transform = anchorCog->has(Transform);
  if (transform == nullptr)
    return;

  Real3 localPosition = transform->TransformPointInverse(position);
  ClothAnchor& anchor = mAnchors.PushBack();
  anchor.mIndex = index;
  anchor.mAnchor = anchorCog;
  anchor.mLocalPosition = localPosition;
}

void ClothSpringSystem::ClearAnchors()
{
  mAnchors.Clear();
}

void ClothSpringSystem::ClearSystem()
{
  ClearParticles();
  ClearSprings();
  ClearFaces();
  ClearUvs();
  ClearAnchors();
}

void ClothSpringSystem::ClearParticles()
{
  mOldPositions.Clear();
  mPositions.Clear();
  mInvMasses.Clear();
  mVelocities.Clear();
  mForces.Clear();
}

void ClothSpringSystem::ClearSprings()
{
  mSprings.Clear();
}

void ClothSpringSystem::ClearFaces()
{
  mFaces.Clear();
}

void ClothSpringSystem::ClearUvs()
{
  mUvs.Clear();
}

void ClothSpringSystem::SetMesh(ZeroEngine::Mesh* mesh)
{
  mMesh = mesh;
}

void ClothSpringSystem::AddUv(Real2Param uv)
{
  mUvs.PushBack(uv);
}

//-----------------------------------------------------------------------------ClothWind
ZilchDefineType(ClothWind, builder, type)
{
  // This is required for component binding
  ZilchBindDestructor();
  ZilchBindConstructor();
  ZilchBindMethod(Initialize);

  // Note: All event connection methods must be bound
  ZilchBindMethod(OnComputeLocalClothForces);
  ZilchBindMethod(OnComputeSpaceClothForces);

  ZilchBindFieldProperty(mActive);
  ZilchBindFieldProperty(mSpaceForce);
  ZilchBindFieldProperty(mWindDirection);
  ZilchBindFieldProperty(mWindStrength);
}

ClothWind::ClothWind()
{
  mWindDirection = Real3(0, 0, -1);
  mWindStrength = 5;
  mActive = true;
  mSpaceForce = false;
}

ClothWind::~ClothWind()
{

}

void ClothWind::Initialize(ZeroEngine::CogInitializer* initializer)
{
  ZeroConnectThisTo(this->GetOwner(), "ComputeClothForces", "OnComputeLocalClothForces");
  ZeroConnectThisTo(this->GetSpace(), "ComputeClothForces", "OnComputeSpaceClothForces");
}

void ClothWind::OnComputeLocalClothForces(ClothSpringSystemEvent* event)
{
  if (!mSpaceForce)
    ComputeClothForces(event->mClothSystem);
}

void ClothWind::OnComputeSpaceClothForces(ClothSpringSystemEvent* event)
{
  if (mSpaceForce)
    ComputeClothForces(event->mClothSystem);
}

void ClothWind::ComputeClothForces(ClothSpringSystem* system)
{
  if (!mActive)
    return;

  int count = system->GetFaceCount();
  for (int i = 0; i < count; ++i)
  {
    Integer3 indices = system->GetFaceIndices(i);
    int indexA = indices.x;
    int indexB = indices.y;
    int indexC = indices.z;
    Real3 posA = system->GetParticlePosition(indexA);
    Real3 posB = system->GetParticlePosition(indexB);
    Real3 posC = system->GetParticlePosition(indexC);

    Real3 normal = Math::Cross(posB - posA, posC - posA);
    normal = Math::Normalized(normal);

    Real forceStrength = Math::Dot(mWindDirection, normal) * mWindStrength;
    Real3 force = Math::Abs(forceStrength) * mWindDirection;

    system->ApplyParticleForce(indexA, force);
    system->ApplyParticleForce(indexB, force);
    system->ApplyParticleForce(indexC, force);
  }
}
