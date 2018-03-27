#include <Core/Animation/Skinning/LinearBlendSkinning.hpp>

namespace Ra {
namespace Core {
namespace Animation {

void linearBlendSkinning( const Vector3Array&  inMesh,
                             const Pose&          pose,
                             const WeightMatrix&  weight,
                             Vector3Array&        outMesh ) {
    outMesh.clear();
    outMesh.resize( inMesh.size(), Vector3::Zero() );
    for( int k = 0; k < weight.outerSize(); ++k ) {
        const int nonZero = weight.col( k ).nonZeros();
        WeightMatrix::InnerIterator it0( weight, k );
        #pragma omp parallel for
        for( int nz = 0; nz < nonZero; ++nz ) {
            WeightMatrix::InnerIterator it = it0 + Eigen::Index(nz);
            const uint   i = it.row();
            const uint   j = it.col();
            const Scalar w = it.value();
            outMesh[i] += w * ( pose[j] * inMesh[i] );
        }
    }
}

void rigidSkinning( const Vector3Array&  inMesh,
                             const Pose&          pose,
                             const WeightMatrix&  weight,
                             Vector3Array&        outMesh ) {
    outMesh.clear();
    outMesh.resize( inMesh.size(), Vector3::Zero() );

    Eigen::VectorXf maxW = Eigen::VectorXf::Zero(inMesh.size());
    Eigen::VectorXi maxJ = Eigen::VectorXi::Zero(inMesh.size());


    for( int k = 0; k < weight.outerSize(); ++k ) {
        const int nonZero = weight.col( k ).nonZeros();
        WeightMatrix::InnerIterator it0( weight, k );
        for( int nz = 0; nz < nonZero; ++nz ) {
            WeightMatrix::InnerIterator it = it0 + Eigen::Index(nz);
            const uint   i = it.row();
            const uint   j = it.col();
            const Scalar w = it.value();

            if (w > maxW[i])
            {
                maxW[i] = w;
                maxJ[i] = j;
            }
        }
    }
    for ( int i = 0; i < inMesh.size(); ++i)
    {
        outMesh[i] = pose[maxJ[i]]* inMesh[i];
    }
  }

} // namespace Animation
} // namespace Core
} // namespace Ra

