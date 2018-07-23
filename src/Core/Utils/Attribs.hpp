#ifndef RADIUMENGINE_ATTRIBS_HPP
#define RADIUMENGINE_ATTRIBS_HPP

#include <Core/Containers/VectorArray.hpp>
#include <Core/Index/IndexMap.hpp>
#include <Core/Log/Log.hpp>
#include <Core/RaCore.hpp>

namespace Ra {
namespace Core {

template <typename T>
class Attrib;

/// AttribBase is the base class for Attrib.
/// An Attrib is data linked to Vertices of a mesh.
/// In the future, it is expected to allow automatic binding between the CPU
/// and the rendered data on the GPU.
class AttribBase {
  public:
    /// attrib name is used to automatic location binding when using shaders.
    virtual ~AttribBase() {}
    std::string getName() const { return m_name; }
    void setName( std::string name ) { m_name = name; }
    virtual void resize( size_t s ) = 0;

    virtual uint getSize() = 0;
    virtual int getStride() = 0;

    bool inline operator==( const AttribBase& rhs ) { return m_name == rhs.getName(); }

    template <typename T>
    inline Attrib<T>& cast() {
        return static_cast<Attrib<T>&>( *this );
    }

    template <typename T>
    inline const Attrib<T>& cast() const {
        return static_cast<const Attrib<T>&>( *this );
    }

    virtual bool isScalar() const = 0;
    virtual bool isVec2() const = 0;
    virtual bool isVec3() const = 0;
    virtual bool isVec4() const = 0;

  private:
    std::string m_name;
};

template <typename T>
class Attrib : public AttribBase {
  public:
    using value_type = T;
    using Container = VectorArray<T>;

    /// resize the container (value_type must have a default ctor).
    void resize( size_t s ) override { m_data.resize( s ); }

    /// RW acces to container data
    inline Container& data() { return m_data; }

    /// R only acccess to container data
    inline const Container& data() const { return m_data; }

    virtual ~Attrib() { m_data.clear(); }
    uint getSize() override { return Container::Matrix::RowsAtCompileTime; }
    int getStride() override { return sizeof( typename Container::value_type ); }

    bool isScalar() const override { return std::is_same<Scalar, T>::value; }
    bool isVec2() const override { return std::is_same<Core::Vector2, T>::value; }
    bool isVec3() const override { return std::is_same<Core::Vector3, T>::value; }
    bool isVec4() const override { return std::is_same<Core::Vector4, T>::value; }

  private:
    Container m_data;
};

template <typename T>
class AttribHandle {
  public:
    typedef T value_type;
    using Container = typename Attrib<T>::Container;

    /// There is no validity check against the corresponding mesh, but just a
    /// simple test to allow the manipuation of unitialized handles.
    constexpr bool isValid() const { return m_idx.isValid(); }

    /// compare two handle, there are the same if they both represent the same
    /// attrib (type and value).
    template <typename U>
    bool operator==( const AttribHandle<U>& lhs ) const {
        return std::is_same<T, U>::value && m_idx == lhs.m_idx;
    }

    Ra::Core::Index idx() { return m_idx; }

  private:
    Ra::Core::Index m_idx = -1;

    friend class AttribManager;
};

/*!
 * \brief The VertexAttribManager provides attributes management by handles
 *
 * The VertexAttribManager stores a container of VertexAttribBase, which can
 * be accessed and deleted (#removeAttrib) using a VertexAttribHandle. Handles
 * are created from an attribute name using #addAttrib, and retrieved using
 * #getAttribHandler.
 *
 * Example of typical use case:
 * \code
 * // somewhere: creation
 * VertexAttribManager mng;
 * auto inputfattrib = mng.addAttrib<Scalar>("MyAttrib");
 *
 * ...
 *
 * // somewhere else: access
 * auto iattribhandler = mng.getAttribHandler<Scalar>("MyAttrib"); //  iattribhandler ==
 * inputfattrib if (iattribhandler.isValid()) { auto &attrib = mng.getAttrib( iattribhandler );
 * }
 * \endcode
 *
 * \warning There is no error check on the handles attribute type
 *
 */
class AttribManager {
  public:
    using value_type = AttribBase*;
    using Container = Ra::Core::IndexMap<value_type>;

    AttribManager() {}

    /// Shallow copy of attributes
    AttribManager( const AttribManager& m ) = default;
    AttribManager& operator=( const AttribManager& m ) = default;

    // Deep copy of attributes
    /// Base copy, does nothing.
    void copyAttribs( const AttribManager& m ) {}

    /// Deep copy of attrib.
    /// Note: if some attrib already exist, it will be replaced but not de-allocated.
    template <class T, class... Handle>
    void copyAttribs( const AttribManager& m, const AttribHandle<T>& attr, Handle... attribs ) {
        if ( attr.isValid() )
        {
            // get attrib to copy
            auto a = m.getAttrib( attr );
            // add new attrib
            auto h = addAttrib<T>( a.getName() );
            getAttrib( h ).data() = a.data();
        }
        // deal with other attribs
        copyAttribs( m, attribs... );
    }

    /// clear all attribs, invalidate handles and shallow copies.
    void clear() {
        for ( auto attr : m_attribs )
        {
            delete attr;
        }
        m_attribs.clear();
        m_attribsIndex.clear();
    }

    /*!
     * \brief getAttrib Grab an attribute handler by name
     * \param name Name of the attribute
     * \return Attribute handler
     * \code
     * VertexAttribManager mng;
     * auto inputfattrib = mng.addAttrib<Scalar>("MyAttrib");
     *
     * auto iattribhandler = mng.getAttribHandler<Scalar>("MyAttrib"); //  iattribhandler ==
     * inputfattrib if (iattribhandler.isValid()) { // true auto &attrib = mng.getAttrib(
     * iattribhandler );
     * }
     * auto& iattribhandler = mng.getAttribHandler<Scalar>("InvalidAttrib"); // invalid
     * if (iattribhandler.isValid()) { // false
     *    ...
     * }
     * \endcode
     * \warning There is no error check on the attribute type
     */
    template <typename T>
    inline AttribHandle<T> getAttribHandle( const std::string& name ) const {
        auto c = m_attribsIndex.find( name );
        AttribHandle<T> handle;
        if ( c != m_attribsIndex.end() )
        {
            handle.m_idx = c->second;
        }
        return handle;
    }

    /// Get attribute by handle
    template <typename T>
    inline Attrib<T>& getAttrib( AttribHandle<T> h ) {
        return *static_cast<Attrib<T>*>( m_attribs.at( h.m_idx ) );
    }

    /// Get attribute by handle (const)
    template <typename T>
    inline const Attrib<T>& getAttrib( AttribHandle<T> h ) const {
        return *static_cast<Attrib<T>*>( m_attribs.at( h.m_idx ) );
    }

    /// Add attribute by name.
    /// Note: If an attribute with the same name already exists,
    ///  it will be replaced but not de-allocated.
    template <typename T>
    AttribHandle<T> addAttrib( const std::string& name ) {
        AttribHandle<T> h;
        Attrib<T>* attrib = new Attrib<T>;
        attrib->setName( name );

        auto it = std::find_if( m_attribs.begin(), m_attribs.end(),
                                [&name]( const auto& a ) { return a->getName() == name; } );

        if ( it != m_attribs.end() )
        {
            LOG( logWARNING ) << "Replacing existing attribute " << name << ".";
            *it = attrib;
            h.m_idx = m_attribsIndex[name];
        } else
        {
            h.m_idx = m_attribs.insert( attrib );
            m_attribsIndex[name] = h.m_idx;
        }

        return h;
    }

    /// Remove attribute by name
    void removeAttrib( const std::string& name ) {
        auto c = m_attribsIndex.find( name );
        if ( c != m_attribsIndex.end() )
        {
            Ra::Core::Index idx = c->second;
            delete m_attribs[idx];
            m_attribs.remove( idx );
            m_attribsIndex.erase( c );
        }
    }

    /// Remove attribute by handle, invalidate all the handles
    template <typename T>
    void removeAttrib( AttribHandle<T> h ) {
        const auto& att = getAttrib( h ); // check the attribute exists
        removeAttrib( att.getName() );
    }

  private:
    /// const acces to attrib vector
    const Container& attribs() const { return m_attribs; }

    // Map between the attrib's name and its index
    std::map<std::string, Ra::Core::Index> m_attribsIndex;

    // Attrib list
    Container m_attribs;

    // Ease wrapper
    friend class TopologicalMesh;
};

} // namespace Core
} // namespace Ra

#endif // RADIUMENGINE_ATTRIBS_HPP
