#include <Core/Algorithm/Subdivision/CatmullClarkSubdivider.hpp>
#include <Core/Algorithm/Subdivision/SubdividerUtils.hpp>

namespace Ra {
namespace Core {

bool CatmullClarkSubdivider::prepare( TopologicalMesh& mesh ) {
    mesh.add_property( m_vpPos );
    mesh.add_property( m_epH );
    mesh.add_property( m_fpH );
    mesh.add_property( m_creaseWeights );
    addPropsCopy( mesh.m_scalarPph, mesh, m_scalarProps );
    addPropsCopy( mesh.m_vec2Pph, mesh, m_vec2Props );
    addPropsCopy( mesh.m_vec3Pph, mesh, m_vec3Props );
    addPropsCopy( mesh.m_vec4Pph, mesh, m_vec4Props );
    addPropsCopyF( mesh.m_scalarPph, mesh, m_scalarPropsF );
    addPropsCopyF( mesh.m_vec2Pph, mesh, m_vec2PropsF );
    addPropsCopyF( mesh.m_vec3Pph, mesh, m_vec3PropsF );
    addPropsCopyF( mesh.m_vec4Pph, mesh, m_vec4PropsF );

    // initialize all weights to 0 (= smooth edge)
    for ( auto e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it )
        mesh.property( m_creaseWeights, *e_it ) = 0.0;

    return true;
}

bool CatmullClarkSubdivider::cleanup( TopologicalMesh& mesh ) {
    mesh.remove_property( m_vpPos );
    mesh.remove_property( m_epH );
    mesh.remove_property( m_fpH );
    mesh.remove_property( m_creaseWeights );
    clearProps( m_scalarProps, mesh );
    clearProps( m_vec2Props, mesh );
    clearProps( m_vec3Props, mesh );
    clearProps( m_vec4Props, mesh );
    clearProps( m_scalarPropsF, mesh );
    clearProps( m_vec2PropsF, mesh );
    clearProps( m_vec3PropsF, mesh );
    clearProps( m_vec4PropsF, mesh );
    return true;
}

bool CatmullClarkSubdivider::subdivide( TopologicalMesh& mesh, size_t n,
                                        const bool update_points ) {
    m_oldVertexOps.clear();
    m_newVertexOps.clear();
    m_oldVertexOps.resize( n );
    m_newVertexOps.resize( n );
    // Do n subdivisions
    for ( size_t iter = 0; iter < n; ++iter )
    {
        // Compute face centroid
        const int NV = mesh.n_vertices();
        m_newVertexOps[iter].reserve( mesh.n_faces() );
#pragma omp parallel for
        for ( uint i = 0; i < mesh.n_faces(); ++i )
        {
            const auto& fh = mesh.face_handle( i );
            // compute centroid
            TopologicalMesh::Point centroid;
            mesh.calc_face_centroid( fh, centroid );
            TopologicalMesh::VertexHandle vh;
#pragma omp critical
            { vh = mesh.new_vertex( centroid ); }
            mesh.property( m_fpH, fh ) = vh;
            // register operation
            const int v = mesh.valence( fh );
            const Scalar inv_v = 1.f / v;
            std::vector<V_OP> ops( v );
            auto heh = mesh.halfedge_handle( fh );
            for ( uint j = 0; j < v; ++j )
            {
                ops[j] = V_OP( inv_v, mesh.to_vertex_handle( heh ) );
                heh = mesh.next_halfedge_handle( heh );
            }
#pragma omp critical
            { m_newVertexOps[iter].push_back( V_OPS( vh, ops ) ); }
            // deal with properties
            interpolateProps( mesh.m_scalarPph, m_scalarPropsF, fh, mesh );
            interpolateProps( mesh.m_vec2Pph, m_vec2PropsF, fh, mesh );
            interpolateProps( mesh.m_vec3Pph, m_vec3PropsF, fh, mesh );
            interpolateProps( mesh.m_vec4Pph, m_vec4PropsF, fh, mesh );
        }

        // Compute position for new (edge-) vertices and store them in the edge property
        m_newVertexOps[iter].reserve( m_newVertexOps[iter].size() + mesh.n_edges() );
#pragma omp parallel for
        for ( uint i = 0; i < mesh.n_edges(); ++i )
        {
            const auto& eh = mesh.edge_handle( i );
            compute_midpoint( mesh, eh, update_points, iter );
        }

        // position updates activated?
        if ( update_points )
        {
            // compute new positions for old vertices
            m_oldVertexOps[iter].reserve( NV );
#pragma omp parallel for
            for ( uint i = 0; i < NV; ++i )
            {
                const auto& vh = mesh.vertex_handle( i );
                update_vertex( mesh, vh, iter );
            }

            // Commit changes in geometry
#pragma omp parallel for
            for ( uint i = 0; i < NV; ++i )
            {
                const auto& vh = mesh.vertex_handle( i );
                mesh.set_point( vh, mesh.property( m_vpPos, vh ) );
            }
        }

        // Split each edge at midpoint stored in edge property m_epPos;
        // Attention! Creating new edges, hence make sure the loop ends correctly.
        auto e_end = mesh.edges_end();
        for ( auto e_itr = mesh.edges_begin(); e_itr != e_end; ++e_itr )
        {
            split_edge( mesh, *e_itr );
        }

        // Commit changes in topology and reconsitute consistency
        // Attention! Creating new faces, hence make sure the loop ends correctly.
        auto f_end = mesh.faces_end();
        for ( auto f_itr = mesh.faces_begin(); f_itr != f_end; ++f_itr )
        {
            split_face( mesh, *f_itr );
        }

        // Commit properties
        commitProps( m_scalarProps, mesh, mesh.m_scalarPph );
        commitProps( m_vec2Props, mesh, mesh.m_vec2Pph );
        commitProps( m_vec3Props, mesh, mesh.m_vec3Pph );
        commitProps( m_vec4Props, mesh, mesh.m_vec4Pph );

#if defined( _DEBUG ) || defined( DEBUG )
        // Now we have an consistent mesh!
        assert( OpenMesh::Utils::MeshCheckerT<TopologicalMesh>( mesh ).check() );
#endif
    }

    // ###########################################################################
    // FIXME: THIS IS ONLY THERE BECAUSE CORE ONLY MANAGES TRIANGLE MESHES FOR NOW

    // triangulate resulting quads
    auto f_end = mesh.faces_end();
    for ( auto f_itr = mesh.faces_begin(); f_itr != f_end; ++f_itr )
    {
        // get all needed halfedges
        auto heh1 = mesh.halfedge_handle( *f_itr );
        auto heh2 = mesh.next_halfedge_handle( heh1 );
        auto heh3 = mesh.next_halfedge_handle( heh2 );
        auto heh4 = mesh.next_halfedge_handle( heh3 );
        auto heh5 = mesh.new_edge( mesh.to_vertex_handle( heh1 ), mesh.to_vertex_handle( heh3 ) );
        auto heh6 = mesh.opposite_halfedge_handle( heh5 );

        // triangulate
        auto fnew = mesh.new_face();
        mesh.set_halfedge_handle( fnew, heh2 );
        mesh.set_face_handle( heh5, *f_itr );
        mesh.set_face_handle( heh2, fnew );
        mesh.set_face_handle( heh3, fnew );
        mesh.set_face_handle( heh6, fnew );
        mesh.set_next_halfedge_handle( heh1, heh5 );
        mesh.set_next_halfedge_handle( heh5, heh4 );
        mesh.set_next_halfedge_handle( heh3, heh6 );
        mesh.set_next_halfedge_handle( heh6, heh2 );

        // deal with properties
        copyProps( m_scalarProps, heh3, heh5, mesh );
        copyProps( m_vec2Props, heh3, heh5, mesh );
        copyProps( m_vec3Props, heh3, heh5, mesh );
        copyProps( m_vec4Props, heh3, heh5, mesh );
        copyProps( m_scalarProps, heh1, heh6, mesh );
        copyProps( m_vec2Props, heh1, heh6, mesh );
        copyProps( m_vec3Props, heh1, heh6, mesh );
        copyProps( m_vec4Props, heh1, heh6, mesh );
    }

    // Commit properties (again since can still subdivide after all this)
    commitProps( m_scalarProps, mesh, mesh.m_scalarPph );
    commitProps( m_vec2Props, mesh, mesh.m_vec2Pph );
    commitProps( m_vec3Props, mesh, mesh.m_vec3Pph );
    commitProps( m_vec4Props, mesh, mesh.m_vec4Pph );

    // ###########################################################################

    // compute normals
    mesh.request_face_normals();
    mesh.request_vertex_normals();
    mesh.update_normals();

    return true;
}

void CatmullClarkSubdivider::split_face( TopologicalMesh& mesh,
                                         const TopologicalMesh::FaceHandle& fh ) {
    /*
        Split an n-gon into n quads by connecting
        each vertex of fh to vh.

        - fh will remain valid (it will become one of the quads)
        - the halfedge handles of the new quads will point to the old halfedges
    */

    // Since edges are already refined (valence*2)
    size_t valence = mesh.valence( fh ) / 2;

    // Add new mesh vertex from face centroid
    auto vh = mesh.property( m_fpH, fh );

    // init new topology with first face
    auto hend = mesh.halfedge_handle( fh );
    auto hh = mesh.next_halfedge_handle( hend );
    auto hold = mesh.new_edge( mesh.to_vertex_handle( hend ), vh );

    mesh.set_next_halfedge_handle( hend, hold );
    mesh.set_face_handle( hold, fh );

    // deal with properties for vh
    copyProps( m_scalarPropsF, m_scalarProps, fh, hold, mesh );
    copyProps( m_vec2PropsF, m_vec2Props, fh, hold, mesh );
    copyProps( m_vec3PropsF, m_vec3Props, fh, hold, mesh );
    copyProps( m_vec4PropsF, m_vec4Props, fh, hold, mesh );

    // go around new vertex to build topology
    hold = mesh.opposite_halfedge_handle( hold );

    // deal with properties for hold
    copyProps( m_scalarProps, hend, hold, mesh );
    copyProps( m_vec2Props, hend, hold, mesh );
    copyProps( m_vec3Props, hend, hold, mesh );
    copyProps( m_vec4Props, hend, hold, mesh );

    for ( size_t i = 1; i < valence; i++ )
    {
        // go to next mid-edge vertex
        auto hnext = mesh.next_halfedge_handle( hh );
        auto hnew = mesh.new_edge( mesh.to_vertex_handle( hnext ), vh );

        // create new face
        auto fnew = mesh.new_face();
        mesh.set_halfedge_handle( fnew, hh );
        mesh.set_face_handle( hnew, fnew );
        mesh.set_face_handle( hold, fnew );
        mesh.set_face_handle( hh, fnew );
        mesh.set_face_handle( hnext, fnew );
        mesh.set_next_halfedge_handle( hnew, hold );
        mesh.set_next_halfedge_handle( hold, hh );

        // deal with properties for hnew
        copyProps( m_scalarPropsF, m_scalarProps, fh, hnew, mesh );
        copyProps( m_vec2PropsF, m_vec2Props, fh, hnew, mesh );
        copyProps( m_vec3PropsF, m_vec3Props, fh, hnew, mesh );
        copyProps( m_vec4PropsF, m_vec4Props, fh, hnew, mesh );

        // prepare for next face
        hh = mesh.next_halfedge_handle( hnext );
        hold = mesh.opposite_halfedge_handle( hnew );
        mesh.set_next_halfedge_handle( hnext, hnew ); // this has to be done after hh !

        // deal with properties for hold
        copyProps( m_scalarProps, hnext, hold, mesh );
        copyProps( m_vec2Props, hnext, hold, mesh );
        copyProps( m_vec3Props, hnext, hold, mesh );
        copyProps( m_vec4Props, hnext, hold, mesh );
    }

    // finish topology
    mesh.set_next_halfedge_handle( hold, hh );
    mesh.set_next_halfedge_handle( hh, hend );
    hh = mesh.next_halfedge_handle( hend );
    mesh.set_next_halfedge_handle( hh, hold );
    mesh.set_face_handle( hold, fh );
    mesh.set_halfedge_handle( vh, hold );
}

void CatmullClarkSubdivider::split_edge( TopologicalMesh& mesh,
                                         const TopologicalMesh::EdgeHandle& eh ) {
    using HeHandle = TopologicalMesh::HalfedgeHandle;
    using VHandle = TopologicalMesh::VertexHandle;
    using FHandle = TopologicalMesh::FaceHandle;

    // prepare data
    HeHandle heh = mesh.halfedge_handle( eh, 0 );
    HeHandle opp_heh = mesh.halfedge_handle( eh, 1 );
    HeHandle new_heh;
    HeHandle opp_new_heh;
    HeHandle t_heh;
    VHandle vh;
    VHandle vh1( mesh.to_vertex_handle( heh ) );

    // new vertex
    vh = mesh.property( m_epH, eh );

    // Re-link mesh entities
    if ( mesh.is_boundary( eh ) )
    {
        for ( t_heh = heh; mesh.next_halfedge_handle( t_heh ) != opp_heh;
              t_heh = mesh.opposite_halfedge_handle( mesh.next_halfedge_handle( t_heh ) ) )
            ;
    } else
    {
        for ( t_heh = mesh.next_halfedge_handle( opp_heh );
              mesh.next_halfedge_handle( t_heh ) != opp_heh;
              t_heh = mesh.next_halfedge_handle( t_heh ) )
            ;
    }

    new_heh = mesh.new_edge( vh, vh1 );
    opp_new_heh = mesh.opposite_halfedge_handle( new_heh );
    mesh.set_vertex_handle( heh, vh );

    mesh.set_next_halfedge_handle( t_heh, opp_new_heh );
    mesh.set_next_halfedge_handle( new_heh, mesh.next_halfedge_handle( heh ) );
    mesh.set_next_halfedge_handle( heh, new_heh );
    mesh.set_next_halfedge_handle( opp_new_heh, opp_heh );

    if ( mesh.face_handle( opp_heh ).is_valid() )
    {
        mesh.set_face_handle( opp_new_heh, mesh.face_handle( opp_heh ) );
        mesh.set_halfedge_handle( mesh.face_handle( opp_new_heh ), opp_new_heh );
    }

    if ( mesh.face_handle( heh ).is_valid() )
    {
        mesh.set_face_handle( new_heh, mesh.face_handle( heh ) );
        mesh.set_halfedge_handle( mesh.face_handle( heh ), heh );
    }

    mesh.set_halfedge_handle( vh, new_heh );
    mesh.set_halfedge_handle( vh1, opp_new_heh );

    // Never forget this, when playing with the topology
    mesh.adjust_outgoing_halfedge( vh );
    mesh.adjust_outgoing_halfedge( vh1 );

    // deal with custom properties
    interpolateProps( m_scalarProps, heh, opp_heh, t_heh, new_heh, opp_new_heh, mesh );
    interpolateProps( m_vec2Props, heh, opp_heh, t_heh, new_heh, opp_new_heh, mesh );
    interpolateProps( m_vec3Props, heh, opp_heh, t_heh, new_heh, opp_new_heh, mesh );
    interpolateProps( m_vec4Props, heh, opp_heh, t_heh, new_heh, opp_new_heh, mesh );
}

void CatmullClarkSubdivider::compute_midpoint( TopologicalMesh& mesh,
                                               const TopologicalMesh::EdgeHandle& eh,
                                               const bool update_points, int iter ) {
    TopologicalMesh::HalfedgeHandle heh = mesh.halfedge_handle( eh, 0 );
    TopologicalMesh::HalfedgeHandle opp_heh = mesh.halfedge_handle( eh, 1 );

    TopologicalMesh::Point pos = mesh.point( mesh.to_vertex_handle( heh ) );
    pos += mesh.point( mesh.to_vertex_handle( opp_heh ) );

    // prepare operations
    std::vector<V_OP> ops;

    // boundary edge: just average vertex positions
    // this yields the [1/2 1/2] mask
    if ( mesh.is_boundary( eh ) || !update_points )
    {
        pos *= 0.5;
        // register operations
        ops.resize( 2 );
        ops[0] = V_OP( 0.5, mesh.to_vertex_handle( heh ) );
        ops[1] = V_OP( 0.5, mesh.to_vertex_handle( opp_heh ) );
    } else // inner edge: add neighbouring Vertices to sum
           // this yields the [1/16 1/16; 3/8 3/8; 1/16 1/16] mask
    {
        pos += mesh.point( mesh.property( m_fpH, mesh.face_handle( heh ) ) );
        pos += mesh.point( mesh.property( m_fpH, mesh.face_handle( opp_heh ) ) );
        pos *= 0.25;
        // register operations
        ops.resize( 4 );
        ops[0] = V_OP( 0.25, mesh.to_vertex_handle( heh ) );
        ops[1] = V_OP( 0.25, mesh.to_vertex_handle( opp_heh ) );
        ops[2] = V_OP( 0.25, mesh.property( m_fpH, mesh.face_handle( heh ) ) );
        ops[3] = V_OP( 0.25, mesh.property( m_fpH, mesh.face_handle( opp_heh ) ) );
    }

#pragma omp critical
    {
        auto vh = mesh.new_vertex( pos );
        mesh.property( m_epH, eh ) = vh;
        m_newVertexOps[iter].push_back( V_OPS( vh, ops ) );
    }
}

void CatmullClarkSubdivider::update_vertex( TopologicalMesh& mesh,
                                            const TopologicalMesh::VertexHandle& vh, int iter ) {
    TopologicalMesh::Point pos( 0.0, 0.0, 0.0 );

    // prepare operations
    std::vector<V_OP> ops;

    // TODO boundary, Extraordinary Vertex and  Creased Surfaces
    // see "A Factored Approach to Subdivision Surfaces"
    // http://faculty.cs.tamu.edu/schaefer/research/tutorial.pdf
    // and http://www.cs.utah.edu/~lacewell/subdeval
    if ( mesh.is_boundary( vh ) )
    {
        pos = mesh.point( vh );
        ops.resize( 3 );
        ops[0] = V_OP( 1.f / 3.f, vh );
        int i = 1;
        for ( auto ve_itr = mesh.ve_iter( vh ); ve_itr.is_valid(); ++ve_itr )
        {
            if ( mesh.is_boundary( *ve_itr ) )
            {
                pos += mesh.point( mesh.property( m_epH, *ve_itr ) );
                ops[i++] = V_OP( 1.f / 3.f, mesh.property( m_epH, *ve_itr ) );
            }
        }
        pos /= 3.0;
    } else // inner vertex
    {
        /* For each (non boundary) vertex V, introduce a new vertex whose
           position is F/n + 2E/n + (n-3)V/n where F is the average of
           the new face vertices of all faces adjacent to the old vertex
           V, E is the average of the midpoints of all edges incident
           on the old vertex V, and n is the number of edges incident on
           the vertex.
       */
        const Scalar valence = mesh.valence( vh );
        const Scalar inv_v2 = 1.f / ( valence * valence );
        ops.resize( valence * 2 + 1 );

        int i = 0;
        for ( auto voh_it = mesh.voh_iter( vh ); voh_it.is_valid(); ++voh_it )
        {
            pos += mesh.point( mesh.to_vertex_handle( *voh_it ) );
            ops[i++] = V_OP( inv_v2, mesh.to_vertex_handle( *voh_it ) );
        }
        pos *= inv_v2;

        TopologicalMesh::Point Q( 0, 0, 0 );
        for ( auto vf_itr = mesh.vf_iter( vh ); vf_itr.is_valid(); ++vf_itr )
        {
            Q += mesh.point( mesh.property( m_fpH, *vf_itr ) );
            ops[i++] = V_OP( inv_v2, mesh.property( m_fpH, *vf_itr ) );
        }

        Q *= inv_v2;

        pos += mesh.point( vh ) * ( valence - 2.0 ) / valence + Q;
        ops[i] = V_OP( ( valence - 2.0 ) / valence, vh );
    }

    // don't set yet since would be needed for other vertices;
    mesh.property( m_vpPos, vh ) = pos;

#pragma omp critical
    { m_oldVertexOps[iter].push_back( V_OPS( vh, ops ) ); }
}

} // namespace Core
} // namespace Ra
