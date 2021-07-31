#ifndef _SAMPLE_DATA_H
#define _SAMPLE_DATA_H

struct SampleData {
  SampleData() :
    count (0), q(NULL), qdot(NULL), qddot(NULL), tau(NULL)
  {}
  ~SampleData() {
    deleteData();
  }
  SampleData(const SampleData &data) {
    count = data.count;

    q = new RigidBodyDynamics::Math::VectorNd[count];
    qdot = new RigidBodyDynamics::Math::VectorNd[count];
    qddot = new RigidBodyDynamics::Math::VectorNd[count];
    tau = new RigidBodyDynamics::Math::VectorNd[count];
    durations = data.durations;

    for (int si = 0; si < count; si++) {
      q[si] = data.q[si];
      qdot[si] = data.qdot[si];
      qddot[si] = data.qddot[si];
      tau[si] = data.tau[si];
    }
  }
  SampleData& operator= (const SampleData &data) {
    if (this != &data) {
      deleteData();
      *this = SampleData (data);
    }
    return *this;
  }

  unsigned int count;
  RigidBodyDynamics::Math::VectorNd *q;
  RigidBodyDynamics::Math::VectorNd *qdot;
  RigidBodyDynamics::Math::VectorNd *qddot;
  RigidBodyDynamics::Math::VectorNd *tau;
  RigidBodyDynamics::Math::VectorNd durations;

  void deleteData() {
    count = 0;

    if (q) {
      delete[] q;
    }
    q = NULL;

    if (qdot) {
      delete[] qdot;
    }
    qdot = NULL;

    if (qddot) {
      delete[] qddot;
    }
    qddot = NULL;

    if (tau) {
      delete[] tau;
    }
    tau = NULL;

    durations.resize(0);
  }

  void fillRandom (int dof_count, int sample_count) {
    deleteData();
    count = sample_count;

    q = new RigidBodyDynamics::Math::VectorNd[count];
    qdot = new RigidBodyDynamics::Math::VectorNd[count];
    qddot = new RigidBodyDynamics::Math::VectorNd[count];
    tau = new RigidBodyDynamics::Math::VectorNd[count];

    for (int si = 0; si < count; si++) {
      q[si].resize (dof_count);
      qdot[si].resize (dof_count);
      qddot[si].resize (dof_count);
      tau[si].resize (dof_count);

      for (int i = 0; i < dof_count; i++) {
        q[si][i] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. -1.;
        qdot[si][i] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. -1.;
        qddot[si][i] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. -1.;
        tau[si][i] = (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 2. -1.;
      }
    }

    durations = RigidBodyDynamics::Math::VectorNd::Zero(count);
  }
};

#endif
