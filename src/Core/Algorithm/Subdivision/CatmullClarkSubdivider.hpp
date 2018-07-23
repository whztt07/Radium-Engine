#ifndef RADIUMENGINE_CATMULLCLARKSUBDIVIDER_H
#define RADIUMENGINE_CATMULLCLARKSUBDIVIDER_H

#include <Core/Mesh/TopologicalTriMesh/TopologicalMesh.hpp>

#include <OpenMesh/Tools/Subdivider/Uniform/SubdividerT.hh>

namespace Ra {
namespace Core {

/// This class implements the Loop subdivision algorithm
///
/// This class extends OpenMesh's CatmullClarkT subdivider to handle attributes.
class RA_CORE_API CatmullClarkSubdivider
    : public OpenMesh::Subdivider::Uniform::SubdividerT<TopologicalMesh, Scalar> {

  public:
    using base = OpenMesh::Subdivider::Uniform::SubdividerT<TopologicalMesh, Scalar>;
    using V_OP = std::pair<Scalar, TopologicalMesh::VertexHandle>;
    using V_OPS = std::pair<TopologicalMesh::VertexHandle, std::vector<V_OP>>;
    using S_OPS = std::vector<V_OPS>;

  public:
    CatmullClarkSubdivider() : base() {}

    CatmullClarkSubdivider( TopologicalMesh& mesh ) : base( mesh ) {}

    ~CatmullClarkSubdivider() {}

  public:
    /// must implement
    const char* name( void ) const override { return "CatmullClarkSubdivider"; }

    /// @return the list of vertex operations done during subdivision.
    const std::vector<S_OPS>& getNewVertexOperations() const { return m_newVertexOps; }
    /// @return the list of vertex operations done during subdivision.
    const std::vector<S_OPS>& getOldVertexOperations() const { return m_oldVertexOps; }

  protected:
    virtual bool prepare( TopologicalMesh& mesh ) override;

    virtual bool cleanup( TopologicalMesh& mesh ) override;

    virtual bool subdivide( TopologicalMesh& mesh, size_t n,
                            const bool update_points = true ) override;

  private:
    // topology helpers

    void split_edge( TopologicalMesh& mesh, const TopologicalMesh::EdgeHandle& eh );

    void split_face( TopologicalMesh& mesh, const TopologicalMesh::FaceHandle& fh );

    void compute_midpoint( TopologicalMesh& mesh, const TopologicalMesh::EdgeHandle& eh,
                           const bool update_points, int iter );

    void update_vertex( TopologicalMesh& mesh, const TopologicalMesh::VertexHandle& vh, int iter );

  private:
    /// old vertex new position
    OpenMesh::VPropHandleT<TopologicalMesh::Point> m_vpPos;

    /// new edge midpoint handle
    OpenMesh::EPropHandleT<TopologicalMesh::VertexHandle> m_epH;

    /// new face point handle
    OpenMesh::FPropHandleT<TopologicalMesh::VertexHandle> m_fpH;

    /// crease weights
    OpenMesh::EPropHandleT<Scalar> m_creaseWeights;

    /// deal with properties
    std::vector<OpenMesh::HPropHandleT<Scalar>> m_scalarProps;
    std::vector<OpenMesh::HPropHandleT<Vector2>> m_vec2Props;
    std::vector<OpenMesh::HPropHandleT<Vector3>> m_vec3Props;
    std::vector<OpenMesh::HPropHandleT<Vector4>> m_vec4Props;
    std::vector<OpenMesh::FPropHandleT<Scalar>> m_scalarPropsF;
    std::vector<OpenMesh::FPropHandleT<Vector2>> m_vec2PropsF;
    std::vector<OpenMesh::FPropHandleT<Vector3>> m_vec3PropsF;
    std::vector<OpenMesh::FPropHandleT<Vector4>> m_vec4PropsF;

    /// list of vertices computations
    std::vector<S_OPS> m_oldVertexOps;
    std::vector<S_OPS> m_newVertexOps;
};

} // namespace Core
} // namespace Ra

#endif // RADIUMENGINE_LOOPSUBDIVIDER_H
